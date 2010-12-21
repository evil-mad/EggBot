#!/usr/bin/perl
use strict;
use warnings;
use Cairo;
use HPGL;
use Getopt::Long;

my $output_file = "plot-preview.png";
my $verbose = 0;

GetOptions(
    "output=s" => \$output_file,
    "verbose+" => \$verbose,
);

# read in the .plt file
my $file = shift @ARGV;
my $plt = HPGL->new($file) or die "Can't read $file\n";

# these are the default pens
my %pens = (
        black => [0, 0, 0], #0
        blue => [0, 0, 1], #1
        red => [1, 0, 0], #2
        green => [0, 1, 0], #3
        magenta => [1, 0, 1], #4
        yellow => [1, 1, 0], #5
        cyan => [0, 1, 1], #6
        brown => [0, 1, 1], #7
        orange => [.7, 0.25, .25], #8
    );

my ($max_x, $max_y) = $plt->size();
print "Preview plot of $file, its size is [$max_x, $max_y]\n";
my $surface = Cairo::ImageSurface->create('argb32', $max_x, $max_y);
my $ctx = Cairo::Context->create($surface);
$ctx->set_source_rgb(1, 1, 1);
$ctx->rectangle(0, 0, $max_x, $max_y);
$ctx->fill();
my $pen_list = $plt->pens();

# put 'black' on at the end of the list
my @pen_order = grep(!/black/, @{$pen_list});
push @pen_order, 'black'; 

foreach my $pen_color (@pen_order) {
    print "Plotting color: $pen_color\n";
    $ctx->set_source_rgb(@{$pens{$pen_color}});
    my $points = $plt->points($pen_color);
    my $cnt = 0;
    foreach my $p (@{$points}) {
        $cnt += 1;
        # account for different plot layout PNG vs EggBot
        my $nx;
        my $ny;
        $nx = $p->[1];
        $ny = $max_y - $p->[2];
        if  ($p->[0] eq 'm') {
            $ctx->move_to($nx, $ny);
        } else {
            $ctx->line_to($nx, $ny);
        }
    }
    print "Plotted $cnt points in that color.\n";
    $ctx->stroke();
}
$surface->write_to_png("plot-preview.png");

# figure out all the line segments in a plot color
sub segments {
    my ($points) = @_;
    my @segs;
    # assume we start from zero 
    my ($cur_x, $cur_y) = (0, 0);
    foreach my $p (@{$points}) {
        if ($p->[0] eq 'd') {
            # create a segment if drawing
            push @segs, [$cur_x, $cur_y, $p->[1], $p->[2]];
        }
        # always update the current cursor
        ($cur_x, $cur_y) = ($p->[1], $p->[2]);
        }
    }
    # now @segs has all line segments we expect
    # to draw
    # Go through them and build connected paths
    my @paths;
    my @segs2;
    while (1) {
        my $p = shift @segs;    # grab first segment
        my ($ex, $ey) = ($p->[2], $p->[3]); # end of segment 
        while (my $n = shift @segs) {
            if ($s2[0] == $s[2] and $s2[1] eq $s[3]) {
            }
        }
    }
}
