package main

import (
	"flag"
	"fmt"
	"gopkg.in/yaml.v3"
	"os"
	"path/filepath"
	"strings"
)

type DeviceDefinition struct {
	Device           string `yaml:"Device"`
	VendorID         string `yaml:"VendorID"`
	VendorName       string `yaml:"VendorName"`
	PID              string `yaml:"PID"`
	Description      string `yaml:"Description"`
	DocumentationURL string `yaml:"DocumentationURL"`
	DeviceType       string `yaml:"DeviceType"`
	Channels         int    `yaml:"Channels"`
	Notes            string `yaml:"Notes"`
}

var pathFlag = flag.String("path", "../../documentation/devices/", "Path to *.yml files for device documentation")

func parsefile(filename string) (*DeviceDefinition, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("unable to read file: %v")
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
	otherfiles := 0

	files, err := os.ReadDir(*pathFlag)
	if err != nil {
		panic(err)
	}

	for _, file := range files {
		entry, err := parsefile(filepath.Join(*pathFlag, file.Name()))
		if err != nil {
			panic(err)
		}

		// The `devicelist.sh` script autogenerates entries
		// with `Device: TODO`.  For now, let's filter them
		// out.
		//
		// Longer-term, we may wish to gripe when we find
		// them, or replace `TODO` with `???` or similar.
		if strings.TrimSpace(entry.Description) != "TODO" {
			entries = append(entries, entry)
		} else {
			otherfiles++
		}
	}

	fmt.Printf("# Devices Supported by LinuxCNC-Ethercat\n")
	fmt.Printf("\n")
	fmt.Printf("*This is a work in progress, listing all of the devices that LinuxCNC-Ethercat\n")
	fmt.Printf("has code to support today.  Not all of these are well-tested.*\n")
	fmt.Printf("\n")

	fmt.Printf("Description | Name in Source Code | EtherCAT VID:PID | Device Type | Channels | Notes\n")
	fmt.Printf("----------- | ------------------- | ---------------- | ----------- | -------: | ------\n")
	for _, e := range entries {
		name := e.Description
		if e.DocumentationURL[0:4] == "http" {
			name = fmt.Sprintf("[%s](%s)", e.Description, e.DocumentationURL)
		}
		channels := fmt.Sprintf("%d", e.Channels)
		if e.Channels == 0 {
			channels = ""
		}
		fmt.Printf("%s | %s | %s:%s | %s | %s | %s\n", name, e.Device, e.VendorID, e.PID, e.DeviceType, channels, e.Notes)
	}

	fmt.Printf("\n")
	if otherfiles > 0 {
		fmt.Printf("There are an additional %d devices supported that do not have enough\n", otherfiles)
		fmt.Printf("documentation to display here.  Please look at the `documentation/devices/` files\n")
		fmt.Printf("and update them if you're able.\n")
	}
}
