#!/usr/bin/env perl
use strict;
use warnings;
use File::Slurp;

#----------------------------------------------------------------------#
# Usage
#----------------------------------------------------------------------#
my ($boot_size_str, $boot_ld, $app_ld) = @ARGV;
die <<"USAGE" unless defined $boot_size_str && defined $boot_ld && defined $app_ld;
Usage:
  $0 <boot_size> <bootloader.ld> <application.ld>

Example:
  $0 64k bootloader/cfg/STM32G474xE.ld application/cfg/STM32G474xE.ld
USAGE

#----------------------------------------------------------------------#
# Parse a length literal like "64k", "0x10000", "512k - 32k" → bytes
#----------------------------------------------------------------------#
sub parse_length {
    my $v = shift;
    $v =~ s/\s//g;
    return hex($v)                     if $v =~ /^0x[0-9a-f]+$/i;
    return int($1*1024)                if $v =~ /^(\d+(?:\.\d+)?)k$/i;
    return int($1*1024*1024)           if $v =~ /^(\d+(?:\.\d+)?)m$/i;
    return int($v)                     if $v =~ /^\d+$/;
    die "Can't parse length literal '$v'";
}

#----------------------------------------------------------------------#
# Format bytes → "Nk" if divisible by 1024, else raw bytes
#----------------------------------------------------------------------#
sub format_length {
    my $b = shift;
    return ($b % 1024)==0
         ? sprintf("%dk", $b/1024)
         : $b;
}

# Taille du bootloader en octets
my $boot_bytes = parse_length($boot_size_str);

#----------------------------------------------------------------------#
# Traitement du bootloader
#----------------------------------------------------------------------#
{
    # File::Slurp: en list context, read_file retourne une ligne par élément
    my @lines = read_file($boot_ld);
    open my $out, '>', "$boot_ld.tmp" or die $!;

    my $found = 0;
    foreach my $line (@lines) {
        if ($line =~ /^(\s*)flash0\s*\([^)]*\)\s*:\s*org\s*=\s*(0x[0-9A-Fa-f]+)\s*,\s*len\s*=\s*[^\s}]+/) {
            my ($ws, $base) = ($1, $2);
            print $out sprintf("%sflash0 (rx) : org = %s, len = %s\n",
                               $ws, $base, $boot_size_str);
            $found = 1;
        }
        else {
            print $out $line;
        }
    }
    close $out;
    die "❌ flash0 not found in $boot_ld\n" unless $found;
    rename("$boot_ld.tmp", $boot_ld) or die $!;
}

#----------------------------------------------------------------------#
# Traitement de l'application
#----------------------------------------------------------------------#
{
    my @lines = read_file($app_ld);
    open my $out, '>', "$app_ld.tmp" or die $!;

    my $found = 0;
    foreach my $line (@lines) {
        if ($line =~ /^(\s*)flash0\s*\([^)]*\)\s*:\s*org\s*=\s*(0x[0-9A-Fa-f]+)\s*\+\s*([^\s,]+)\s*,\s*len\s*=\s*([^\s}]+)/) {
            my ($ws, $base, $old_off_str, $old_len_expr) = ($1, $2, $3, $4);

            # extrait FLASH_TOTAL_LITERAL (avant le '-')
            my ($flash_total_literal) = $old_len_expr =~ /^\s*([^\s-]+)/;

            # génère la nouvelle ligne
            print $out sprintf(
                "%sflash0 (rx) : org = %s + %s, len = %s - %s\n",
                $ws, $base, $boot_size_str,
                $flash_total_literal, $boot_size_str
            );
            $found = 1;
        }
        else {
            print $out $line;
        }
    }
    close $out;
    die "❌ flash0 not found in $app_ld\n" unless $found;
    rename("$app_ld.tmp", $app_ld) or die $!;
}

print "✅ flash0 lines regenerated with boot size $boot_size_str\n";
