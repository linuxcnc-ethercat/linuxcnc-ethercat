module github/linuxcnc-ethercat/linuxcnc-ethercat/scripts/deviceequivs

go 1.21.5
toolchain go1.23.7

require (
	github.com/linuxcnc-ethercat/esi-data/esi v0.0.0-20240108172128-4830b35d83ae
	gopkg.in/yaml.v3 v3.0.1
)

require (
	golang.org/x/net v0.36.0 // indirect
	golang.org/x/text v0.22.0 // indirect
)

replace github.com/linuxcnc-ethercat/esi-data/esi => /home/scott/git/esi-data/esi
