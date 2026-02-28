#!/usr/bin/env perl
use strict;
use warnings;

use Getopt::Long qw(GetOptions);

my $elf = q{};
my $objdump = q{};

GetOptions(
    'elf=s'     => \$elf,
    'objdump=s' => \$objdump,
) or die "Usage: $0 --elf <file.elf> [--objdump <objdump-bin>]\n";

if ($elf eq q{}) {
    die "Missing --elf argument\n";
}

if ($objdump eq q{}) {
    $objdump = 'arm-none-eabi-objdump';
}

open my $fh, '-|', $objdump, '-h', $elf
  or die "Cannot execute '$objdump -h $elf': $!\n";

my @sections;
my $pending = undef;

while (my $line = <$fh>) {
    chomp $line;

    if ($line =~ /^\s*\d+\s+(\S+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+/) {
        my ($name, $size_hex, $vma_hex, $lma_hex) = ($1, $2, $3, $4);
        $pending = {
            name  => $name,
            size  => hex($size_hex),
            vma   => hex($vma_hex),
            lma   => hex($lma_hex),
        };
        next;
    }

    if (defined $pending && $line =~ /^\s*([A-Z, ]+)$/) {
        my $flags = $1;
        my $is_alloc = ($flags =~ /\bALLOC\b/) ? 1 : 0;
        my $is_load  = ($flags =~ /\bLOAD\b/)  ? 1 : 0;
        if ($is_alloc && $pending->{size} > 0) {
            push @sections, {
                name      => $pending->{name},
                size      => $pending->{size},
                vma_start => $pending->{vma},
                vma_end   => $pending->{vma} + $pending->{size},
                lma_start => $pending->{lma},
                lma_end   => $pending->{lma} + $pending->{size},
                is_load   => $is_load,
            };
        }
        $pending = undef;
    }
}

close $fh;

sub fmt_hex {
    my ($value) = @_;
    return sprintf('0x%08X', $value);
}

sub overlaps_for {
    my ($list_ref, $start_key, $end_key) = @_;
    my @list = sort { $a->{$start_key} <=> $b->{$start_key} } @{$list_ref};
    my @overlaps;

    for (my $i = 0; $i < @list; $i++) {
        for (my $j = $i + 1; $j < @list; $j++) {
            last if $list[$j]->{$start_key} >= $list[$i]->{$end_key};
            if ($list[$j]->{$start_key} < $list[$i]->{$end_key}) {
                push @overlaps, [$list[$i], $list[$j]];
            }
        }
    }

    return @overlaps;
}

my @vma_overlaps = overlaps_for(\@sections, 'vma_start', 'vma_end');
my @load_sections = grep { $_->{is_load} } @sections;
my @lma_overlaps = overlaps_for(\@load_sections, 'lma_start', 'lma_end');

if (!@vma_overlaps && !@lma_overlaps) {
    exit 0;
}

print "ELF overlap check FAILED for '$elf'\n";

if (@vma_overlaps) {
    print "VMA overlaps:\n";
    for my $pair (@vma_overlaps) {
        my ($a, $b) = @{$pair};
        print "  $a->{name} [" . fmt_hex($a->{vma_start}) . ".." . fmt_hex($a->{vma_end} - 1)
          . "] overlaps $b->{name} [" . fmt_hex($b->{vma_start}) . ".."
          . fmt_hex($b->{vma_end} - 1) . "]\n";
    }
}

if (@lma_overlaps) {
    print "LMA overlaps:\n";
    for my $pair (@lma_overlaps) {
        my ($a, $b) = @{$pair};
        print "  $a->{name} [" . fmt_hex($a->{lma_start}) . ".." . fmt_hex($a->{lma_end} - 1)
          . "] overlaps $b->{name} [" . fmt_hex($b->{lma_start}) . ".."
          . fmt_hex($b->{lma_end} - 1) . "]\n";
    }
}

exit 2;
