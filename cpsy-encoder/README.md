# cpsi-encoder

A C++ server running an encoder library.

## Dependencies

You will need the development files to lgpio and libtbb.

> sudo apt install libtbb-dev liblgpio-dev

## Compiling

Just run `make`.

> make

## Installing

Run `make install` as root.

> sudo make install

This will install the server and its systemd service files.
Your server will be started on every boot.

## Usage

When running the server, you find a socket file `/tmp/encoder.sock`.
Test it with ncat.

> ncat -u -U /tmp/encoder.sock

It will reply with two comma-separated numbers to each message.
The first number is for what you configure as Motor A.
The second one is for Motor B.

## Configuration

Change the constants in `encoderd.cpp` to match your pins.
The defaults are 6 and 5 for Motor A and 24 and 23 for
Motor B.

You can also change the update rate. The default is 40 times
per second.
