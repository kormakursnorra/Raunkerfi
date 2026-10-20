CXXFLAGS=-Wall -Werror -O2 -flto
LDLIBS=-ltbb -latomic -llgpio

.DEFAULT: all
all: encoder encoderd

encoder: encoder.hpp encoder.cpp standalone.cpp

encoderd: encoder.hpp encoder.cpp encoderd.cpp


.PHONY: install
install: encoderd encoderd.service
	-make uninstall
	install -m 0755 -o root -g root -s encoderd /usr/local/bin
	install -m 0644 -o root -g root encoderd.service /etc/systemd/system/
	systemctl daemon-reload
	systemctl enable encoderd.service
	systemctl start encoderd.service

.PHONY: uninstall
uninstall:
	systemctl stop encoderd.service
	systemctl disable encoderd.service
	rm -f /usr/local/bin/encoderd
	rm -f /etc/systemd/system/encoderd.service
	systemctl daemon-reload
