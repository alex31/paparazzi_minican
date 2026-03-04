#!/usr/bin/env perl
use strict;
use warnings;
use Getopt::Long qw(GetOptions);
use Time::HiRes qw(sleep);
use File::Basename qw(basename);
use File::Spec;

my ($file, $count, $wait, $device, $help);

GetOptions(
    'file|f=s'   => \$file,
    'count|n=i'  => \$count,
    'wait|w=f'   => \$wait,
    'device|d=s' => \$device,
    'help|h'     => \$help,
) or usage();

usage() if $help;
usage('Missing --file')   unless defined $file;
usage('Missing --count')  unless defined $count;
usage('Missing --wait')   unless defined $wait;
usage('Missing --device') unless defined $device;
usage('Count must be > 0')      unless $count > 0;
usage('Wait time must be >= 0') unless $wait >= 0;
usage("File '$file' not found or unreadable") unless -r $file;

my $payload = do {
    open my $fh, '<:raw', $file or die "Unable to open file '$file': $!\n";
    local $/;
    my $data = <$fh>;
    close $fh;
    $data;
};

my $ref_path = File::Spec->catfile('/tmp', basename($file) . '.concatenated_reference');
open my $ref, '>:raw', $ref_path or die "Unable to open reference file '$ref_path': $!\n";

open my $dev, '>>:raw', $device or die "Unable to open device '$device': $!\n";

for my $idx (1 .. $count) {
    my $written = print {$dev} $payload;
    die "Write failed on '$device': $!\n" unless $written;
    my $ref_written = print {$ref} $payload;
    die "Write failed on reference file '$ref_path': $!\n" unless $ref_written;

    warn "Sent $idx/$count frames\r" if -t STDOUT;
    sleep($wait) if $idx < $count && $wait > 0;
}

print "Completed sending $count frames to $device\nReference file: $ref_path\n" if -t STDOUT;

close $dev or warn "Close failed on '$device': $!\n";
close $ref or warn "Close failed on reference file '$ref_path': $!\n";
exit 0;

sub usage {
    my ($msg) = @_;
    warn "$msg\n" if defined $msg;
    warn <<'USAGE';
Usage: send_can_file.pl --file <path> --count <n> --wait <seconds> --device <tty>
  -f, --file     File to send
  -n, --count    Number of repetitions (> 0)
  -w, --wait     Wait time between sends (seconds, can be fractional, >= 0)
  -d, --device   Target tty (e.g., /dev/ttyUSB0)
  -h, --help     Show this help
USAGE
    exit(1);
}
