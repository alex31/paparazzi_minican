#!/usr/bin/perl
use strict;
use warnings;
use Fcntl qw(:seek);
use Digest::CRC qw(crc);

my $data = "123456789";


# Magic number to identify a valid firmware header
use constant FIRMWARE_HEADER_MAGIC => 0xF0CACC1A;

# --- Main ---
if (@ARGV != 4) {
    print "Usage: perl prepend_header.pl <in_bin> <out_bin> <board_name> <hw_ver_major>\n";
    print "Example: perl prepend_header.pl in.bin out.bin MySensor-V1 1\n";
    exit(1);
}

my ($input_bin, $output_bin, $board_name, $hw_ver_major) = @ARGV;

# Read the original firmware binary
open(my $in_fh, '<:raw', $input_bin) or die "Cannot open input file $input_bin: $!";
my $firmware_data;
{
    local $/; # Enable "slurp" mode to read the whole file
    $firmware_data = <$in_fh>;
}
close($in_fh);

my $firmware_size = length($firmware_data);
# CRC-32K/4.2 (Koopman *op*)
my $firmware_crc = crc($firmware_data,
              32,           # width
              0xFFFFFFFF,   # init
              0xFFFFFFFF,   # xorout
              1,            # refout
              0xC9D204F5,   # poly 
              1,            # refin
              0);           # cont 


printf("Input file:         %s\n", $input_bin);
printf("Board Name:         %s\n", $board_name);
printf("HW Version Major:   %d\n", $hw_ver_major);
printf("Firmware size:      %d bytes\n", $firmware_size);
printf("Firmware CRC32:     %08X\n", $firmware_crc);


# Create the header using Perl's pack function.
# 'V' is a 32-bit unsigned integer, little-endian.
# 'a32' is a null-padded string of 32 bytes.
my $header = pack(
    'V a32 V V V',
    FIRMWARE_HEADER_MAGIC,
    $board_name,
    $hw_ver_major,
    $firmware_size,
    $firmware_crc);

# Write the header and firmware to the output file
open(my $out_fh, '>:raw', $output_bin) or die "Cannot open output file $output_bin: $!";
print $out_fh $header;
print $out_fh $firmware_data;
close($out_fh);

my $output_size = -s $output_bin;
printf("\nSuccessfully created %s (%d bytes)\n", $output_bin, $output_size);
 
