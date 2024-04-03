package main

import (
	"bytes"
	"flag"
	"fmt"
	"gopkg.in/yaml.v3"
	"log"
	"os"
	"os/exec"
	"sort"
	"strings"
)

type DeviceDefinition struct {
	Device    string
	VendorID  string
	PID       string
	ModParams []string
}

var (
	devicesFlag = flag.String("devices_path", "../lcec_devices", "Path to lcec_devices tool")
	outputFile  = flag.String("output", "configgen/drivers.go", "File to create")
)

func parsefile(filename string) (*DeviceDefinition, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("unable to read file: %v", filename)
	}

	entry := &DeviceDefinition{}

	err = yaml.Unmarshal(data, entry)
	if err != nil {
		return nil, fmt.Errorf("unable to decode yaml: %v", err)
	}

	return entry, nil
}

func main() {
	flag.Parse()

	entries := []*DeviceDefinition{}

	data, err := exec.Command(*devicesFlag).Output()
	if err != nil {
		log.Fatalf("Unable to execute %q: %v", *devicesFlag, err)
	}

	// the output is small enough that this isn't a big deal.
	lines := strings.Split(string(data), "\n")

	sort.Strings(lines)
	for _, line := range lines {
		pieces := strings.Split(line, "\t")
		if len(pieces) > 2 {
			entry := &DeviceDefinition{
				Device:   pieces[0],
				VendorID: pieces[1],
				PID:      pieces[2],
			}
			
			if pieces[4] != "" {
				entry.ModParams = strings.SplitAfter(pieces[4], "> ")
			}
			
			entries = append(entries, entry)
		}
	}

	var sb bytes.Buffer

	sb.WriteString("// Code generated  DO NOT EDIT.\n")
	sb.WriteString("package configgen\n")
	sb.WriteString("type EthercatDriver struct {\n")
	sb.WriteString("    VendorID string\n")
	sb.WriteString("    ProductID string\n")
	sb.WriteString("    Type string\n")
	sb.WriteString("    ModParams []string\n")
	sb.WriteString("}\n")

	sb.WriteString("var Drivers=[]EthercatDriver{\n")
	for _, e := range entries {
		sb.WriteString(fmt.Sprintf("  EthercatDriver{\n"))
		sb.WriteString(fmt.Sprintf("    VendorID: %q,\n", e.VendorID))
		sb.WriteString(fmt.Sprintf("    ProductID: %q,\n", e.PID))
		sb.WriteString(fmt.Sprintf("    Type: %q,\n", e.Device))
		sb.WriteString(fmt.Sprintf("    ModParams: []string{\n"))
		for _, mp := range e.ModParams {
			sb.WriteString(fmt.Sprintf("      %q,\n", mp))
		}
		sb.WriteString(fmt.Sprintf("    },\n"))
		sb.WriteString(fmt.Sprintf("  },\n"))
	}
	sb.WriteString("}\n")

	err = os.WriteFile(*outputFile, sb.Bytes(), 0644)
	if err != nil {
		panic(err)
	}
}
