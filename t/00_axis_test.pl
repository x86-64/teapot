#!/usr/bin/perl 

use common;
use Time::HiRes qw(usleep);

device_prepare();

my $i=0;
for (1..60) {
	device_write("G0 X".($i/100)."\n");
	$l = device_read();
	#$l =~ /ok/ or die;
	
	#usleep(500000);
	sleep(2);
	$i++;
}
device_write("G0 X0\n");
device_read();

device_close();
