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

