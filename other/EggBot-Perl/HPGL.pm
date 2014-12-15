# Simple HPGL interperter/reader for EggBot with Cairo preview
# mode (saves on eggs)
# Written Dec-2010 Chuck McManis
use strict;
use warnings;

package HPGL;

BEGIN {
    require Exporter;
    my @ISA = qw( Exporter );
    my @EXPORT_OK = qw( new );
}

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

#                      0     1   2    3     4       5     6     7     8
my @pen_colors = qw( black blue red green magenta yellow cyan brown orange);


# this creates an in memory image of the HPGL plot
# we assume that the plot was done with a lower left
# hand origin (all positive numbers)
sub new {
    my $self = shift;
    my $class = ref ($self) || $self;
    
    my ($file, $verbose) = @_;
    $self = bless {}, $class;
    my %penplots;
    my $current_pen;
    my $points_array;
    my $points = 0;
    my ($xmin, $xmax, $ymin, $ymax) = (0, 0, 0, 0);

    open (my $fh, "<", $file) or die "No such file $!\n";
    while (my $line = <$fh>) {
        chomp($line);
        my ($cmd, $arg) = $line =~ /([A-Z]{2})(.*);/;
        print "Command '$cmd' argument '$arg'\n" if $verbose;
        if ($cmd eq "SP") {
            ($current_pen) = $arg =~/(\d+)/;
            $current_pen -= 1;
            last if ($current_pen == -1);
            $current_pen = $pen_colors[$current_pen];
            print "current pen set to $current_pen\n" if $verbose;
            if (not defined $penplots{$current_pen}) {
                $penplots{$current_pen} = [];
            }
            $points_array = $penplots{$current_pen};
            next;
        }
        next if (($cmd ne "PU") and ($cmd ne "PD"));
        die "no current pen\n" if (!defined $current_pen);

        my ($x, $y) = $arg =~ /([\-0-9]+)\s+([\-0-9]+)/;
        if ($cmd eq "PU") {
            push @{$points_array}, ['m', $x, $y];
        } elsif ($cmd eq "PD") {
            push @{$points_array}, ['d', $x, $y];
        }
        $points += 1;
        print " [ $x, $y] = '$line'\n" if $verbose;
        $xmin = $x if ($x < $xmin);
        $ymin = $y if ($y < $ymin);
        $xmax = $x if ($x > $xmax);
        $ymax = $y if ($y > $ymax);
    }
    $self->{'plots'} = \%penplots;
    $self->{'points'} = $points;
    $self->{'size'} = [$xmax, $ymax];
    $self->{'origin'} = [0, 0]; # reserved
    $self->{'scale'} = 1.0;
    $self->{'size'} = [$xmax - $xmin, $ymax - $ymin];
    close $fh;
    return $self;
}

# Return a list of pens used in this plot
sub pens {
    my ($self) = @_;
    my @used_pens = sort keys %{$self->{'plots'}};
    return \@used_pens;
}

# return the drawing/moving commands for this pen
sub points {
    my ($self, $pen) = @_;
    return $self->{'plots'}{$pen};
}

# return the plot 'size' in plot units
sub size {
    my ($self) = @_;
    my @d = @{$self->{'size'}};
    return @d;
}
