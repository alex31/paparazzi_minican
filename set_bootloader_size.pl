#!/usr/bin/env perl
use strict;
use warnings;
use File::Slurp;

my ($expected_size_str, $boot_ld, $app_ld) = @ARGV;
die <<"USAGE" unless $expected_size_str && $boot_ld && $app_ld;
Usage:
  $0 <expected_size> <bootloader.ld> <application.ld>

Examples:
  $0 64k bootloader/cfg/STM32G474xE.ld application/cfg/STM32G474xE.ld
USAGE

sub parse_length {
    my $len = shift;
    if ($len =~ /^0x[0-9a-f]+$/i) {
        return hex($len);
    } elsif ($len =~ /^(\d+(?:\.\d+)?)k$/i) {
        return int($1 * 1024);
    } elsif ($len =~ /^(\d+(?:\.\d+)?)m$/i) {
        return int($1 * 1024 * 1024);
    } elsif ($len =~ /^\d+$/) {
        return int($len);
    } else {
        die "Unrecognized length format: '$len'";
    }
}

sub extract_flash0_length {
    my $file = shift;
    my $text = read_file($file);
    if ($text =~ /flash0\s*\([^)]*\)\s*:\s*org\s*=\s*[^,]+,\s*len\s*=\s*([^\s]+)/) {
        return ($1, $text);
    }
    die "Couldn't find flash0 length in $file";
}

sub extract_application_offset {
    my $file = shift;
    my $text = read_file($file);
    if ($text =~ /flash0\s*\([^)]*\)\s*:\s*org\s*=\s*0x08000000\s*\+\s*([^\s,]+)/) {
        return ($1, $text);
    }
    die "Couldn't find flash0 origin offset in $file";
}

sub update_file {
    my ($file, $text) = @_;
    write_file($file, $text);
    print "🔧 Updated $file\n";
}

# parse expected size
my $expected_bytes = parse_length($expected_size_str);

# bootloader.ld
my ($boot_val, $boot_text) = extract_flash0_length($boot_ld);
my $boot_bytes = parse_length($boot_val);

# application.ld
my ($app_val, $app_text) = extract_application_offset($app_ld);
my $app_bytes = parse_length($app_val);

# compare
if ($boot_bytes == $expected_bytes && $app_bytes == $expected_bytes) {
    print "✅ Bootloader size is up to date: $expected_bytes bytes\n";
    exit 0;
}

# replace only if needed
if ($boot_bytes != $expected_bytes) {
    $boot_text =~ s/(flash0\s*\([^)]*\)\s*:\s*org\s*=\s*[^,]+,\s*len\s*=\s*)([^\s]+)/$1$expected_size_str/;
    update_file($boot_ld, $boot_text);
}

if ($app_bytes != $expected_bytes) {
    $app_text =~ s/(flash0\s*\([^)]*\)\s*:\s*org\s*=\s*0x08000000\s*\+\s*)([^\s,]+)/$1$expected_size_str/;
    update_file($app_ld, $app_text);
}

print "✅ Bootloader size updated to $expected_size_str ($expected_bytes bytes)\n";
