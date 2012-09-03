use Device::SerialPort::Arduino;

our $Arduino;

sub device_prepare {
	$Arduino = Device::SerialPort::Arduino->new(
		port     => '/dev/ttyUSB1',
		baudrate => 115200,
		databits => 8,
		parity   => 'none',
	);
}

sub device_read {
	my $l = $Arduino->receive();
	print "<<$l\n";
	return $l;
}
sub device_write {
	my $l = shift;
	print ">>$l\n";
	$Arduino->communicate($l);
}

sub device_close {
}

1;
