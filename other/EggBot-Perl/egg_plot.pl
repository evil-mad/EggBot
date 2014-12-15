#!/usr/bin/perl
use strict;
use warnings;

use EggBot;
use HPGL;
use Getopt::Long;

my $dev = "/dev/ttyACM0";
my $test_num = -1;
my $limit = "3200,1000";
my $verbose = 0;
my $flip = 0;

GetOptions(
    'port|device=s' => \$dev,
    'test=i' => \$test_num,
    'limits=s' => \$limit,
    'debug+' => \$verbose,
    'flip+' => \$flip,
);

print "EggPlot v0.5, Dec 2010 by C. McManis\n";
my $egg = EggBot->new(device => $dev, 
                        debug => $verbose, 
                        flip => $flip,
                        limits => $limit)
        || die "Cannot talk to the EggBot.\n";

print "EggBot found!\n";
$egg->motor_off(); 
$egg->pen_up();
my $pi = 3.141592654;
my $deg2rad = $pi / 180.0;

if ($test_num == 1) {
    print "Test 1:\n Move pen to home position.\n";
    print "Then hit enter when ready to plot.\n";
    my $line = <STDIN>;
    $egg->motor_on();
    $egg->set_home();
    test1();
    exit 0;
} elsif ($test_num == 2) {
    print "Test 2:\n Move pen to home position.\n";
    print "Then hit enter when ready to plot.\n";
    my $line = <STDIN>;
    $egg->motor_on();
    $egg->set_home();
    test2();
    exit 0;
}

my $filename = shift @ARGV;
die "Nothing to plot" if (not defined $filename);
print "Reading $filename ...";
my $plt = HPGL->new($filename);
if (defined $plt) {
    my ($w, $h) = $plt->size();
    print "Done.\n Move pen to home position.\n";
    print "Then hit enter when ready to plot.\n";
    print "Plot is $w wide by $h tall.\n";
    print "Note output is FLIPPED.\n" if $flip;
    my $line = <STDIN>;
    $egg->motor_on();
    $egg->set_home();
    print "Read in $filename.\n";
    print "It uses the following pens: \n";
    my $pens = $plt->pens();
    my @pen_list = grep (!/black/, @{$pens});
    push @pen_list, "black";
    foreach my $p (@pen_list) {
        my $points = $plt->points($p);
        $egg->pen_up();
        print "Please put in the $p colored pen and press enter.\n";
        my $xx = <STDIN>;
        print "Plotting ", scalar @{$points}," points in $p\n";
        foreach my $cmd (@{$points}) {
            my ($nx, $ny) = ($cmd->[1], $cmd->[2]);

            if ($cmd->[0] eq 'm') {
                $egg->move_to($nx, $ny);
            } elsif ($cmd->[0] eq 'd') {
                $egg->draw_to($nx, $ny);
            }
        }
    }
    $egg->pen_up();
    $egg->motor_off();
}
exit 0;
# Test 2 really checks to see if the pen up/down
# logic is working correctly. It draws a few
# circles of varying size.
#
test2();
$egg->motor_off();
$egg->close();
sub test2 {
    my @circles = (
        [50, 50, 50],
        [100, 400, 50],
        [300, 600, 100],
        [400, 500, 75],
        [475, 425, 50],
        [525, 375, 25]
    );
    foreach my $c (@circles) {
        circle(@{$c});
    }
}

# Test 1 draws lines back and forth the full Y travel
# and completely around the egg/ball. It gives a good
# example of linearity of the setup.
sub test1 {
    my @lines = (
        [0, 0],
        [0, 800],
        [200, 800],
        [200, 0]
    );
    for (my $x = 0; $x < 3200; $x += 400) {
        foreach my $p (@lines) {
            my $err = $egg->draw_to($x+$p->[0], $p->[1]);
            if ($err) {
                print "Eggbot returned an error.\n";
                $egg->motor_off();
                exit 1;
            }
        }
    }
}

# utility routine to draw circles as 36 segments.
sub circle {
    my ($x, $y, $r) = @_;

    print "Draw circle @ [$x, $y] of radius $r\n";
    if ($egg->move_to($x+$r, $y)) {
        print "EggBot error.\n";
        $egg->motor_off();
        exit(1);
    }
    for (my $i = 0; $i <= 360; $i += 10) {
        my ($nx, $ny);
        $nx = $x + int ($r * cos($i*$deg2rad) + 0.5);
        $ny = $y + int ($r * sin($i*$deg2rad) + 0.5);
        if ($egg->draw_to($nx, $ny)) {
            print "EggBot error.\n";
            $egg->motor_off();
            exit(1);
        }
    }
}
