#!/usr/bin/perl
# command processor for Egg Bot

use strict;
use warnings;
use Getopt::Long;
use EggBot qw( new );

my $verbose = 1;

GetOptions( 
    "--verbose+" => \$verbose,
);

print "Egg Bot Commander.\n";
my $egg = EggBot->new(debug => 1, limits => "3200,1000");
print "EggBot not found!\n" if (! defined $egg);
exit 1 if (! defined $egg);

print "Opened access to the Egg Bot\n";

while (1) {
    print "enter something : ";
    my $thing = <STDIN>;
    chomp($thing);
    next if (not defined $thing);
    print "attempting to send command $thing...\n";
    my $result = $egg->_do_cmd($thing);
    last if ($thing =~ /exit/);
}

$egg->close();

