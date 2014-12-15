#!/usr/bin/perl
# testing SVG support in perl
use strict;
use warnings;
use SVG;
use SVG::Parser;

my $svg= SVG->new(width=>3200, height=>800);
$svg->circle(id=>'my_circle', cx=>100, cy=>100, r=>50);
my $out = $svg->xmlify;
print $out;

my $parser = SVG::Parser->new(-debug => 1);

my $xml;
{
  local $/=undef;
  open (my $fh, "<", "three-boxes.svg");
  $xml = <$fh>;
  close $fh;
}
my $svg2 = $parser->parse($xml);
print "Parsed the svg file\n";
my $gc = $svg2->getChildren();
print "Fetched children from svg they were:\n";
foreach my $c (@{$gc}) {
    tree_walk($c, "");
}
my $element_count;

print "Printed out $element_count elements.\n";


sub tree_walk {
    my ($el, $ndent) = @_;
    my $children;
    $element_count += 1;
    $children = $el->{'-childs'};
    if (defined $children) {
        print "Node has ", scalar @{$children}," children.\n";
    }
    foreach my $ele (@{$children}) {
        tree_walk($ele, $ndent . "   ");
    } 
    foreach my $k (keys %{$el}) {
        print "$ndent $k: --> $el->{$k}\n";
    }
}
