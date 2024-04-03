package main

// This is intended to create a LinuxCNC XML config file on stdout,
// based on information from `ethercat slaves` and `ethercat sdos`.
// It can recognize all EtherCAT devices that LinuxCNC currently has
// drivers for, as well as recognizing otherwise-unknown CiA 402
// devices.  Unknown CiA 402 devices will have their SDOs probed, as
// well as the value of 0x6502:00, and a set of `<modParam>`s will be
// generated that should enable all of the standard CiA 402 features
// that the hardware supports.  This isn't as good as a
// device-specific driver (there's no support for setting
// device-specific options, for example), but it should be usable in
// most cases.

//go:generate ./devicelist --output drivers/drivers.go
//
// This tells Go to generate configgen/drivers.go using the
// 'devicelist' tool in this directory.  This parses all of the YAML
// files in `documentation/devices` to identify drivers, VIDs, and
// PIDs.

import (
	"bufio"
	"bytes"
	"encoding/xml"
	"flag"
	"fmt"
	"github.com/linuxcnc-ethercat/linuxcnc-ethercat/configgen/drivers"
	"os/exec"
	"regexp"
	"strconv"
	"strings"
)

type EthercatSlave struct {
	Master     string
	Slave      string
	VendorID   string
	ProductID  string
	RevisionNo string
	DeviceName string
	SDOs       map[string]string
}

// This block of `Config*` types are intended to match the full set of
// XML parameters that `lcec_conf` can parse, so we can use this for
// editing configs.  For various reasons, that may never actually be
// useful (it'd eat comments, for one), but it's probably worth having
// a second implementation, anyway.
type ConfigRoot struct {
	XMLName xml.Name `xml:"masters"`
	Masters []*ConfigMaster
}

type ConfigMaster struct {
	XMLName           xml.Name `xml:"master"`
	Idx               string   `xml:"idx,attr"`
	AppTimePeriod     string   `xml:"appTimePeriod,attr,omitempty"`
	RefClockSyncCyles string   `xml:"refClockSyncCycles,attr,omitempty"`
	Slaves            []ConfigSlave
}

type ConfigSlave struct {
	Comment      string   `xml:",comment"`
	XMLName      xml.Name `xml:"slave"`
	Idx          string   `xml:"idx,attr"`
	Type         string   `xml:"type,attr"`
	Vid          string   `xml:"vid,attr,omitempty"`
	Pid          string   `xml:"pid,attr,omitempty"`
	Name         string   `xml:"name,attr"`
	ConfigPDOs   string   `xml:"configPdos,attr,omitempty"`
	SyncManagers []*ConfigSyncManager
	ModParams    []interface{}
}

type ConfigModParam struct {
	XMLName xml.Name `xml:"modParam"`
	Name    string   `xml:"name,attr"`
	Value   string   `xml:"value,attr"`
}

type ConfigSyncManager struct {
	XMLName xml.Name `xml:"syncManager"`
	Idx     string   `xml:"idx,attr"`
	Dir     string   `xml:"dir,attr"`
	PDOs    []*ConfigPDO
}

type ConfigPDO struct {
	Comment string   `xml:",comment"`
	XMLName xml.Name `xml:"pdo"`
	Idx     string   `xml:"idx,attr"`
	Entries []*ConfigPDOEntry
}

type ConfigPDOEntry struct {
	XMLName        xml.Name `xml:"pdoEntry"`
	Idx            string   `xml:"idx,attr"`
	SubIdx         string   `xml:"subIdx,attr"`
	BitLen         string   `xml:"bitLen,attr"`
	HalPin         string   `xml:"halPin,attr"`
	HalType        string   `xml:"halType,attr"`
	Scale          string   `xml:"scale,attr,omitempty"`
	Offset         string   `xml:"offset,attr,omitempty"`
	ComplexEntries []*ConfigComplexEntry
	Comment        string `xml:",comment"`
}

type ConfigComplexEntry struct {
	XMLName xml.Name `xml:"complexEntry"`
	BitLen  string   `xml:"bitLen,attr"`
	HalPin  string   `xml:"halPin,attr"`
	HalType string   `xml:"halType,attr"`
	Scale   string   `xml:"scale,attr"`
	Offset  string   `xml:"offset,attr"`
}

type EnableSDO struct {
	offset, subindex int
	name             string
}

type Comment string

func (cm Comment) MarshalXML(e *xml.Encoder, start xml.StartElement) error {
	if cm != "" {
		// This is a terrible hack; the Go XML encoder writes
		// comments immediately after the previous tag, which
		// looks terrible for human-edited config files.  We
		// emit a newline followed by 6 spaces, which ends up
		// making things line up.
		err := e.EncodeToken(xml.CharData("\n      "))
		if err != nil {
			return err
		}
		return e.EncodeToken(xml.Comment(cm))
	}
	return nil
}

type BlankLine string

func (bl BlankLine) MarshalXML(e *xml.Encoder, start xml.StartElement) error {
	return e.EncodeToken(xml.CharData("\n      "))
}

var (
	// NOTE: this list does not include everything; I'm
	// deliberaterly leaving out pins that are mandatory if the
	// device supports their mode, like `actual_position`.
	EnableSDOs = []EnableSDO{
		EnableSDO{name: "enableActualCurrent", offset: 0x78, subindex: 0},
		EnableSDO{name: "enableActualFollowingError", offset: 0xf4, subindex: 0},
		EnableSDO{name: "enableActualTorque", offset: 0x77, subindex: 0},
		EnableSDO{name: "enableActualVelocitySensor", offset: 0x69, subindex: 0},
		EnableSDO{name: "enableActualVoltage", offset: 0x79, subindex: 0},
		EnableSDO{name: "enableControlEffort", offset: 0xfa, subindex: 0},
		EnableSDO{name: "enableDigitalInput", offset: 0xfd, subindex: 0},
		EnableSDO{name: "enableDigitalOutput", offset: 0xfe, subindex: 1},
		EnableSDO{name: "enableErrorCode", offset: 0x3f, subindex: 0},
		EnableSDO{name: "enableFollowingErrorTimeout", offset: 0x66, subindex: 0},
		EnableSDO{name: "enableFollowingErrorWindow", offset: 0x65, subindex: 0},
		EnableSDO{name: "enableHomeAccel", offset: 0x9a, subindex: 0},
		EnableSDO{name: "enableInterpolationTimePeriod", offset: 0xc2, subindex: 1},
		EnableSDO{name: "enableMaximumAcceleration", offset: 0xc6, subindex: 0},
		EnableSDO{name: "enableMaximumCurrent", offset: 0x73, subindex: 0},
		EnableSDO{name: "enableMaximumDeceleration", offset: 0xc6, subindex: 0},
		EnableSDO{name: "enableMaximumMotorRPM", offset: 0x80, subindex: 0},
		EnableSDO{name: "enableMaximumSlippage", offset: 0xf8, subindex: 0},
		EnableSDO{name: "enableMaximumTorque", offset: 0x72, subindex: 0},
		EnableSDO{name: "enableMotorRatedCurrent", offset: 0x75, subindex: 0},
		EnableSDO{name: "enableMotorRatedTorque", offset: 0x76, subindex: 0},
		EnableSDO{name: "enablePolarity", offset: 0x7e, subindex: 0},
		EnableSDO{name: "enablePositionDemand", offset: 0x62, subindex: 0},
		EnableSDO{name: "enablePositioningTime", offset: 0x68, subindex: 0},
		EnableSDO{name: "enablePositioningWindow", offset: 0x67, subindex: 0},
		EnableSDO{name: "enableProbeStatus", offset: 0xb9, subindex: 0},
		EnableSDO{name: "enableProfileAccel", offset: 0x83, subindex: 0},
		EnableSDO{name: "enableProfileDecel", offset: 0x84, subindex: 0},
		EnableSDO{name: "enableProfileEndVelocity", offset: 0x82, subindex: 0},
		EnableSDO{name: "enableProfileMaxVelocity", offset: 0x7f, subindex: 0},
		EnableSDO{name: "enableProfileVelocity", offset: 0x81, subindex: 0},
		EnableSDO{name: "enableTargetTorque", offset: 0x71, subindex: 0},
		EnableSDO{name: "enableTargetVL", offset: 0x42, subindex: 0},
		EnableSDO{name: "enableTorqueDemand", offset: 0x74, subindex: 0},
		EnableSDO{name: "enableTorqueProfileType", offset: 0x88, subindex: 0},
		EnableSDO{name: "enableTorqueSlope", offset: 0x87, subindex: 0},
		EnableSDO{name: "enableVLAccel", offset: 0x48, subindex: 0},
		EnableSDO{name: "enableVLDecel", offset: 0x49, subindex: 0},
		EnableSDO{name: "enableVLDemand", offset: 0x43, subindex: 0},
		EnableSDO{name: "enableVLMaximum", offset: 0x46, subindex: 2},
		EnableSDO{name: "enableVLMinimum", offset: 0x46, subindex: 1},
		EnableSDO{name: "enableVelocityDemand", offset: 0x6b, subindex: 0},
		EnableSDO{name: "enableVelocityErrorTime", offset: 0x6e, subindex: 0},
		EnableSDO{name: "enableVelocityErrorWindow", offset: 0x6d, subindex: 0},
		EnableSDO{name: "enableVelocitySensorSelector", offset: 0x6a, subindex: 0},
		EnableSDO{name: "enableVelocityThresholdTime", offset: 0x70, subindex: 0},
		EnableSDO{name: "enableVelocityThresholdWindow", offset: 0x6f, subindex: 0},
	}

	// Regexes for 'ethercat slaves'
	slaveMasterRE     = regexp.MustCompile("^=== Master ([0-9]+), Slave ([0-9]+) ===$")
	slaveVendorRE     = regexp.MustCompile("^  Vendor Id: +(0x[0-9a-fA-F]+)")
	slaveProductRE    = regexp.MustCompile("^  Product code: +(0x[0-9a-fA-F]+)")
	slaveRevisionRE   = regexp.MustCompile("^  Revision number: +(0x[0-9a-fA-F]+)")
	slaveDeviceNameRE = regexp.MustCompile("^  Device name: (.*)")

	// Regex for 'ethercat sdos'
	sdoRE = regexp.MustCompile("^  (0x[0-9a-fA-F]{4}:[0-9a-fA-F]{2}), [rw-]+, ([^,]+),")

	// Regexes for 'ethercat pdos'
	pdoSMRE    = regexp.MustCompile("^SM([0-9]+): PhysAddr (0x[0-9a-f]+).*")
	pdoPDORE   = regexp.MustCompile("^  ([RT]xPDO) (0x[0-9a-f]+) \"(.*)\"")
	pdoEntryRE = regexp.MustCompile("^    PDO entry (0x[0-9a-f]+):([0-9a-f]+), +([0-9]+) bit, \"(.*)\"")

	// Regex for pin names.  Anything that this matches will be replaced by a `-` in pin names.
	pinRE = regexp.MustCompile("[^a-z0-9]+")

	// Regexes for `lcec_devices`output
	ld_modparamRE = regexp.MustCompile("<modParam name=\"([^\"]+)\" value=\"([^\"]+)\".*")
	ld_commentRE  = regexp.MustCompile("<!--(.*)-->")

	// Sequence number for device naming
	deviceSequence int

	typedbFlag      = flag.Bool("typedb", true, "Use the built-in list of supported EtherCAT device types?  If false, all devices will be 'generic' or 'basic_cia402'.")
	extraciamodFlag = flag.Bool("extra_cia_modparams", false, "Add CiA 402 <modParam>s to all CiA 402 devices, not just 'basic_cia402'.")
	genericPdoFlag  = flag.Bool("generic_pdos", true, "Attempt to build PDOs for generic devices.")
)

// readSlaves calls `ethercat -v slaves` and parses the output,
// returning a slice of EthercatSlave objects.
func readSlaves() ([]EthercatSlave, error) {
	slaves := []EthercatSlave{}

	out, err := exec.Command("ethercat", "-v", "slaves").Output()

	if err != nil {
		return nil, err
	}

	scanner := bufio.NewScanner(bytes.NewReader(out))

	slave := EthercatSlave{}
	for scanner.Scan() {
		line := scanner.Text()

		if slaveMasterRE.MatchString(line) {
			if slave.Slave != "" {
				// The previous device didn't have a "Device Name" field (hi, Rovix ESD-A6!), so we'll have to catch it here.

				slaves = append(slaves, slave)
				slave = EthercatSlave{}
			}
			results := slaveMasterRE.FindStringSubmatch(line)
			slave.Master = string(results[1])
			slave.Slave = string(results[2])
		} else if slaveVendorRE.MatchString(line) {
			results := slaveVendorRE.FindStringSubmatch(line)
			slave.VendorID = string(results[1])
		} else if slaveProductRE.MatchString(line) {
			results := slaveProductRE.FindStringSubmatch(line)
			slave.ProductID = string(results[1])
		} else if slaveRevisionRE.MatchString(line) {
			results := slaveRevisionRE.FindStringSubmatch(line)
			slave.RevisionNo = string(results[1])
		} else if slaveDeviceNameRE.MatchString(line) {
			results := slaveDeviceNameRE.FindStringSubmatch(line)
			slave.DeviceName = string(results[1])

			slaves = append(slaves, slave)
			slave = EthercatSlave{}
		}
	}

	if err := scanner.Err(); err != nil {
		return nil, err
	}

	return slaves, nil
}

// readSDOs calls `ethercat sdos` for the current slave and extracts a
// list of all of the SDOs reported.  These are then added to slave.SDOs.
func (s *EthercatSlave) readSDOs() error {
	sdos := make(map[string]string)

	out, err := exec.Command("ethercat", "-m", s.Master, "sdos", "-p", s.Slave).Output()

	if err != nil {
		return err
	}

	scanner := bufio.NewScanner(bytes.NewReader(out))
	for scanner.Scan() {
		line := scanner.Text()

		if sdoRE.MatchString(line) {
			results := sdoRE.FindStringSubmatch(line)
			sdos[results[1]] = results[2]
		}
	}

	s.SDOs = sdos
	return nil
}

// Returns true if the device is a CiA 402 device.  This probes all 3
// of the SDOs that my copy of the spec says are required; if all are
// present, then we'll assume that it's a CiA 402 device.
func (s *EthercatSlave) isCiA402() bool {
	if (s.SDOs["0x6040:00"] != "") && (s.SDOs["0x6041:00"] != "") && (s.SDOs["0x6502:00"] != "") {
		return true
	}
	return false
}

func readSDO(master, slave string, index, subindex int) int64 {
	out, err := exec.Command("ethercat", "-m", master, "upload", "-p", slave, fmt.Sprintf("0x%04x", index), fmt.Sprintf("0x%02x", subindex)).Output()
	if err != nil {
		panic(err)
	}

	split := strings.Split(string(out), " ")
	value, err := strconv.ParseInt(split[0], 0, 32)
	if err != nil {
		panic(err)
	}

	return value
}

// Identifies the number of channels (or axes) supported by the
// device.  Unlike `isCiA402()`, this only looks for 0x6502.
func (s *EthercatSlave) CiAChannels() int {
	channels := 0
	for _, sdo := range []string{"0x6502:00", "0x6d02:00", "0x7502:00", "0x7d02:00", "0x8502:00", "0x8d02:00", "0x9502:00", "0x9d02:00"} {
		if s.SDOs[sdo] != "" {
			channels++
		} else {
			return channels
		}
	}
	return channels
}

// Number of RX PDO entries on this device.
func (s *EthercatSlave) CiARxPDOEntries() int {
	for entry := 32; entry > 8; entry-- {
		if s.SDOs[fmt.Sprintf("0x1600:%02d", entry)] != "" {
			return entry
		}
	}
	return 8
}

// Number of TX PDO entries on this device.
func (s *EthercatSlave) CiATxPDOEntries() int {
	for entry := 32; entry > 8; entry-- {
		if s.SDOs[fmt.Sprintf("0x1a00:%02d", entry)] != "" {
			return entry
		}
	}
	return 8
}

// CiAEnableModParams looks at the SDOs gathered earlier as well as
// the output of `ethercat upload 0x6502 0` and emits a set of
// `<modParam>` settings that will tell the CiA 402 drivers which
// features to enable for this slave.
//
// This should really only be needed for `basic_cia402`.
func (s *EthercatSlave) CiAEnableModParams() []interface{} {
	mp := []interface{}{}

	channels := s.CiAChannels()

	if channels > 1 {
		//mp = append(mp, Comment("This device supports multiple axes"))
		mp = append(mp, ConfigModParam{Name: "ciaChannels", Value: fmt.Sprintf("%d", channels)})
	}
	mp = append(mp, ConfigModParam{Name: "ciaRxPDOEntryLimit", Value: strconv.Itoa(s.CiARxPDOEntries())})
	mp = append(mp, ConfigModParam{Name: "ciaTxPDOEntryLimit", Value: strconv.Itoa(s.CiATxPDOEntries())})

	for channel := 0; channel < channels; channel++ {
		prefix := ""
		if channels > 1 {
			prefix = fmt.Sprintf("ch%d", channel+1)
		}
		base := 0x6000 + 0x800*channel

		abilities := readSDO(s.Master, s.Slave, base+0x502, 0)

		if (abilities & (1 << 0)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enablePP", Value: "true"})
		}
		if (abilities & (1 << 1)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enableVL", Value: "true"})
		}
		if (abilities & (1 << 2)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enablePV", Value: "true"})
		}
		if (abilities & (1 << 3)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enableTQ", Value: "true"})
		}
		if (abilities & (1 << 5)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enableHM", Value: "true"})
		}
		if (abilities & (1 << 6)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enableIP", Value: "disabled"})
		}
		if (abilities & (1 << 7)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enableCSP", Value: "true"})
		}
		if (abilities & (1 << 8)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enableCSV", Value: "true"})
		}
		if (abilities & (1 << 9)) != 0 {
			mp = append(mp, ConfigModParam{Name: prefix + "enableCST", Value: "true"})
		}

		for _, feature := range EnableSDOs {
			sdo := fmt.Sprintf("0x%04x:%02x", feature.offset+base, feature.subindex)
			if s.SDOs[sdo] != "" {
				mp = append(mp, ConfigModParam{Name: prefix + feature.name, Value: "true"})
			}

			// Add additional modparams next to specific `enable` lines as needed.
			if feature.name == "enableDigitalInput" {
				mp = append(mp, ConfigModParam{Name: prefix + "digitalInChannels", Value: "16"})
			}
			if feature.name == "enableDigitalOutput" {
				mp = append(mp, ConfigModParam{Name: prefix + "digitalOutChannels", Value: "16"})
			}
		}
	}

	return mp
}

func BuildInferMap() map[string]map[string]string {
	r := make(map[string]map[string]string) // vendorID : deviceID : device type
	if *typedbFlag {
		for _, v := range configgen.Drivers {
			if r[v.VendorID] == nil {
				r[v.VendorID] = make(map[string]string)
			}
			r[v.VendorID][v.ProductID] = v.Type
		}
	}

	return r
}

// InferType determines the best 'type=""` value to use in the generated
// XML file.  It looks at which VID:PID pairs current drivers support,
// and if no matches are found it returns either `basic_cia402` or
// `generic`.
func (s *EthercatSlave) InferType(infermap map[string]map[string]string) string {
	if infermap[s.VendorID] != nil && infermap[s.VendorID][s.ProductID] != "" {
		return infermap[s.VendorID][s.ProductID]
	}

	if s.isCiA402() {
		return "basic_cia402"
	} else {
		return "generic"
	}
}

func (s *EthercatSlave) ConfigModParams() []interface{} {
	for _, v := range configgen.Drivers {
		if v.VendorID == s.VendorID && v.ProductID == s.ProductID {
			if len(v.ModParams) > 0 {
				mp := []interface{}{}

				for _, entry := range v.ModParams {
					if ld_modparamRE.MatchString(entry) {
						results := ld_modparamRE.FindStringSubmatch(entry)
						m := &ConfigModParam{
							Name:  results[1],
							Value: results[2],
						}
						mp = append(mp, m)
						mp = append(mp, BlankLine(""))
					} else if ld_commentRE.MatchString(entry) {
						results := ld_commentRE.FindStringSubmatch(entry)
						mp = append(mp, Comment(results[1]))
					}
				}

				return mp
			} else {
				return []interface{}{}
			}
		}
	}
	return []interface{}{}
}

// InferName comes up with a plausible `name=""` value for use in the generated XML file.
func (s *EthercatSlave) InferName() string {
	deviceSequence++
	return fmt.Sprintf("D%d", deviceSequence)
}

func xmlFormatHex(in string) string {
	return strings.ReplaceAll(in, "0x", "")
}

func pinFormatComment(in string) string {
	return pinRE.ReplaceAllString(strings.ToLower(in), "-")
}

func (s *EthercatSlave) BuildPDOs(c *ConfigSlave) error {
	out, err := exec.Command("ethercat", "-m", s.Master, "pdos", "-p", s.Slave).Output()

	var sm *ConfigSyncManager
	var pdo *ConfigPDO

	if err != nil {
		return err
	}

	scanner := bufio.NewScanner(bytes.NewReader(out))

	for scanner.Scan() {
		line := scanner.Text()

		if pdoSMRE.MatchString(line) {
			results := pdoSMRE.FindStringSubmatch(line)
			sm = &ConfigSyncManager{
				Idx: xmlFormatHex(results[1]),
			}

			// Set up bogus defaults for in/out.  I can't
			// figure out how to determine this from
			// `ethercat pdos` in general.  When we have
			// input or output PDOs, then it's easy, but
			// for empty PDOs it's not.  I'm also unsure
			// if we care.
			if sm.Idx == "0" {
				sm.Dir = "in"
			} else if sm.Idx == "1" {
				sm.Dir = "out"
			}

			c.SyncManagers = append(c.SyncManagers, sm)
		} else if pdoPDORE.MatchString(line) {
			results := pdoPDORE.FindStringSubmatch(line)
			pdo = &ConfigPDO{
				Idx:     xmlFormatHex(results[2]),
				Comment: results[3],
			}
			sm.PDOs = append(sm.PDOs, pdo)
			if results[1] == "RxPDO" {
				sm.Dir = "out"
			} else if results[1] == "TxPDO" {
				sm.Dir = "in"
			}
		} else if pdoEntryRE.MatchString(line) {
			results := pdoEntryRE.FindStringSubmatch(line)

			entry := &ConfigPDOEntry{
				Idx:     xmlFormatHex(results[1]),
				SubIdx:  xmlFormatHex(results[2]),
				BitLen:  results[3],
				Comment: results[4],
			}

			// There's no point in even bothering to emit "gap" entries here.
			if entry.Idx == "0000" {
				continue
			}

			entry.HalPin = fmt.Sprintf("pin-%s-%s", xmlFormatHex(entry.Idx), entry.SubIdx)
			if entry.Comment != "" {
				entry.HalPin = pinFormatComment(entry.Comment)
				entry.Comment = ""
			}

			sdotype := s.SDOs[fmt.Sprintf("0x%s:%s", entry.Idx, entry.SubIdx)]
			bits, _ := strconv.ParseUint(entry.BitLen, 0, 32)

			if bits < 8 {
				entry.HalType = "bit"
			} else {
				switch sdotype {
				case "uint8", "uint16", "uint32":
					entry.HalType = "u32"
				case "int8", "int16", "int32":
					entry.HalType = "s32"
				case "bool":
					if bits == 1 {
						entry.HalType = "bit"
					} else {
						entry.HalType = "u32"
					}
				case "uint64": // Some support in LinuxCNC, but not in LCEC today, so it'll throw an error.
					entry.HalType = "u64"
				case "int64": // Some support in LinuxCNC, but not in LCEC today, so it'll throw an error.
					entry.HalType = "s64"
				case "float", "double":
					if bits == 32 {
						entry.HalType = "float-ieee"
					} else {
						entry.HalType = "float-double-ieee"
					}
				case "":
					entry.HalType = "BLANK"
					// should probably just do 'continue' here and skip this pin.
				default:
					// Not sure what to do with this...
					entry.HalType = "!!" + sdotype + "!!"
				}
			}
			pdo.Entries = append(pdo.Entries, entry)
		}
	}

	return nil
}

func checkSlaveDuplicateNames(slave ConfigSlave) map[string]bool {
	names := make(map[string]int)

	for _, sm := range slave.SyncManagers {
		for _, pdo := range sm.PDOs {
			for _, entry := range pdo.Entries {
				names[entry.HalPin]++
			}
		}
	}

	result := make(map[string]bool)
	for k, v := range names {
		if v > 1 {
			result[k] = true
		}
	}

	return result
}

func fixupPinNames(slave ConfigSlave) {
	duplicateNames := checkSlaveDuplicateNames(slave)
	var prefix string

	for _, sm := range slave.SyncManagers {
		for _, pdo := range sm.PDOs {
			split := strings.Split(pdo.Comment, " ")
			if len(split) > 1 {
				prefix = split[len(split)-1] // get the last component of the PDO name
			} else {
				prefix = sm.Dir
			}
			for _, entry := range pdo.Entries {
				if duplicateNames[entry.HalPin] {
					entry.HalPin = pinFormatComment(prefix + " " + entry.HalPin)
				}
			}
		}
	}
}

func main() {
	flag.Parse()

	slaves, err := readSlaves()
	if err != nil {
		panic(err)
	}

	masters := map[string]*ConfigMaster{}
	infermap := BuildInferMap()
	for _, slave := range slaves {
		if masters[slave.Master] == nil {
			masters[slave.Master] = &ConfigMaster{
				Idx:    slave.Master,
				Slaves: []ConfigSlave{},
			}
		}

		err = slave.readSDOs()
		if err != nil {
			panic(err)
		}
		slaveconfig := ConfigSlave{
			Idx:       slave.Slave,
			Type:      slave.InferType(infermap),
			Name:      slave.InferName(),
			ModParams: slave.ConfigModParams(),
		}

		if slaveconfig.Type == "basic_cia402" || (*extraciamodFlag && slave.isCiA402()) {
			slaveconfig.ModParams = append(slaveconfig.ModParams, slave.CiAEnableModParams()...)
		}

		if slaveconfig.Type == "generic" || slaveconfig.Type == "basic_cia402" {
			slaveconfig.Vid = slave.VendorID
			slaveconfig.Pid = slave.ProductID
			slaveconfig.Comment = slave.DeviceName
		}

		if slaveconfig.Type == "generic" && *genericPdoFlag {
			slave.BuildPDOs(&slaveconfig)
		}

		fixupPinNames(slaveconfig)
		masters[slave.Master].Slaves = append(masters[slave.Master].Slaves, slaveconfig)

	}

	r := ConfigRoot{
		Masters: []*ConfigMaster{},
	}

	for _, v := range masters {
		r.Masters = append(r.Masters, v)
	}

	b, err := xml.MarshalIndent(r, "", "  ")
	if err != nil {
		panic(err)
	}

	// Hack to switch from <foo></foo> to <foo/> without breaking <!-- bar --></foo>
	re := regexp.MustCompile("([^-])></[a-zA-Z]+>")
	ns := re.ReplaceAllString(string(b), "$1/>")
	fmt.Println(ns)
}
