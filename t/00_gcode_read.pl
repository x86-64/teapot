#!/usr/bin/perl 

use common;

device_prepare();

foreach (1..30){
	device_write("M400\n");
	$l = device_read();
	$l =~ /ok/ or die;
	sleep 1;
}

device_close();
