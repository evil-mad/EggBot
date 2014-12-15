# Perl library to talk with the EggBot
# written Dec-2010 by Chuck McManis
# Should be portable to any Perl 5.10 system with a POSIX package
# (which should be all of them)

use strict;
use warnings;

package EggBot;

BEGIN {
    require Exporter;
    our @ISA = qw( Exporter );
    our @EXPORT_OK = qw( new );
}

use POSIX qw( :termios_h );
use Fcntl;
use Time::HiRes qw( sleep usleep );
use Getopt::Long;

sub new {
    my $self = shift;
    my $class = ref ($self) || $self;
    my (%opts) = @_;
    $self = bless {}, $class;

    $self->{'flip'} = 0;
    if (defined($opts{'flip'})) {
        $self->{'flip'} = $opts{'flip'};
    }
    $self->{'draw_speed'} = 250;
    if (defined $opts{'draw_speed'}) {
        $self->{'draw_speed'} = $opts{'draw_speed'};
    }
    $self->{'move_speed'} = 600;
    if (defined $opts{'move_speed'}) {
        $self->{'move_speed'} = $opts{'move_speed'};
    }
    $self->{'debug'} = 0;
    if (defined $opts{'debug'}) {
        $self->{'debug'} = $opts{'debug'};
    }
    $self->{'error'} = 0;   # set to true if last command errored out
    $self->{'sync'} = 0;    # set to true when sync'd up

    # try to puzzle out if an EggBot is attached. This is
    # done by starting at unit 0 and walking up to unit 9
    # checking to see if a sync command works.
    my $unit = 0;
    while (1) {
        $self->{'device'} = "/dev/ttyACM$unit";
        if (sysopen ($self->{'fh'}, $self->{'device'}, O_RDWR | O_NDELAY)) {
            # check to see if an EggBot is listening ...
            last if $self->resync();
            close $self->{'fh'};
        }
        $unit += 1;
        return if ($unit > 10);
    }
    print "Device $self->{device} connected to EggBot.\n";

    $self->{'fd'} = fileno ($self->{'fh'});
    _setup_fd($self->{'fd'});

    $self->{'max_x'} = 3200;
    $self->{'max_y'} = 800;
    if (defined $opts{'limits'}) {
        my ($mx, $my) = $opts{'limits'} =~ /(\d+),(\d+)/;
        ($self->{'max_x'}, $self->{'max_y'}) = ($mx, $my);
    }
    $self->{'pen_high'} = 25_000;
    $self->{'pen_low'} = 20_000;
    $self->{'pen_speed'} = 100;
    $self->{'pen_up_speed'} = 500;
    $self->{'pen_delay'} = [.5, .75]; # up, down delay in S
    $self->resync();
    if (! $self->{'sync'}) {
        warn "Unable to sync with EggBot!\n";
        return;
    }
    $self->_do_cmd("sc,4,$self->{pen_high}");
    $self->_do_cmd("sc,5,$self->{pen_low}");
    $self->_do_cmd("sc,10,$self->{pen_speed}");
    $self->_do_cmd("sc,11,$self->{pen_up_speed}");
    $self->{'pen_state'} = 0;
    $self->pen_up();
    return $self;
}

=head2 resync()

Synchronize with the EBB on the EggBot. Send the 'v' (version)
command up to four times trying to get the EggBot to respond
with the firmware version string. String should start with
EBBv13_and_above or the sync will fail.

Returns true if resync was successful. Note when the instance
is created it comes up in "sync", you only need to call this
after a command has returned an error.
=cut
sub resync {
    my ($self) = @_;
    my $retries = 4;
    while ($retries > 0) {
        my ($e, $m) = $self->_do_cmd('v', 1);
        last if ($e == 0);
        $retries -= 1;
    }
    $self->{'sync'} = ($retries > 0);
    $self->{'error'} = 0;
    return $self->{'sync'};
}

=head2 close( ) 

Close the access to the eggbot. Object cannot be 
re-used after this.
=cut
sub close {
    my ($self) = @_;
    close $self->{'fh'};
}

=head2 motor_off()

Turn off the holding current to the stepper motors (they
can be moved freely then and they will not be too hot)
=cut
sub motor_off {
    my ($self) = @_;

    if (! $self->{'sync'}) {
        warn "Can't call motor_off: EggBot not sync'd\n";
        return;
    }
    my ($e, undef) = $self->_do_cmd('em,0,0');
    return $e;
}

=head2 motor_on()

Enable the holding current to the stepper motors.
=cut
sub motor_on {
    my ($self) = @_;

    if (! $self->{'sync'}) {
        warn "Can't call motor_on: EggBot not sync'd\n";
        return;
    }
    my ($e, undef) = $self->_do_cmd('em,1,1');
    return $e;
}

=head2 pen_up()

Put the pen on the EggBot into the 'up' position.
=cut
sub pen_up {
    my ($self) = @_;

    if (! $self->{'sync'}) {
        warn "Can't call pen_up: EggBot not sync'd\n";
        return;
    }
    return 0 if ($self->{'pen_state'} == 1);
    my ($e, undef) = $self->_do_cmd('sp,1');
    $self->{'pen_state'} = 1;
    print "Pen sleep ($self->{pen_delay}[1])\n" if $self->{'debug'};
    sleep($self->{'pen_delay'}[1]);
    return $e;
}

=head2 pen_down()

Put the pen on the EggBot into the 'down' position.
=cut
sub pen_down {
    my ($self) = @_;

    if (! $self->{'sync'}) {
        warn "Can't call pen_down: EggBot not sync'd\n";
        return;
    }
    return 0 if ($self->{'pen_state'} == 0);
    my ($e, $m) = $self->_do_cmd('sp,0');
    $self->{'pen_state'} = 0;
    print "Pen sleep ($self->{pen_delay}[0])\n" if $self->{'debug'};
    sleep($self->{'pen_delay'}[0]);
    return $e;
}

=head2 set_home()

Set the 'home' (or 0,0) position for the current plot. Basically
this resets the internal notion of where the EggBot thinks its
currently drawn to, to 0,0. If you call home() it will send the
plot back to this point.
=cut
sub set_home {
    my ($self) = @_;
    $self->{'cur_x'} = 0;
    $self->{'cur_y'} = 0;
}

=head2 set_max(x, y)

Set the "maximum" extension of X and Y to these values.
=cut
sub set_max {
    my ($self, $x, $y) = @_;
    $self->{'max_x'} = $x;
    $self->{'max_y'} = $y;
}

=head2 move_to(x, y)

Move the plotter (with the pen up) to absolute location (x, y)
=cut
sub move_to {
    my ($self, $x, $y) = @_;

    $self->pen_up();
    $x = $self->{'max_x'} if ($x > $self->{'max_x'});
    $x = 0 if ($x < 0);
    $y = $self->{'max_y'} if ($y > $self->{'max_y'});
    $y = 0 if ($y < 0);
    if ($self->{'flip'}) {
        $x *= -1;
        $y *= -1;
    }
    my $dx = $x - $self->{'cur_x'};
    my $dy = $y - $self->{'cur_y'};
    my $dist = sqrt ($dx * $dx + $dy * $dy);
    my $delay = int ($dist * 1000.0 / $self->{'move_speed'});
    $self->{'cur_x'} = $x;
    $self->{'cur_y'} = $y;
    my ($e, $m) = $self->_do_cmd("sm,$delay,$dy,$dx");
    $delay *= .95;
    usleep($delay * 1000);
    return $e;
}

=head2 draw_to(x, y)

Move the plotter (with the pen down) to absolute location (x, y)
=cut
sub draw_to {
    my ($self, $x, $y) = @_;

    $self->pen_down();
    $x = $self->{'max_x'} if ($x > $self->{'max_x'});
    $x = 0 if ($x < 0);
    $y = $self->{'max_y'} if ($y > $self->{'max_y'});
    $y = 0 if ($y < 0);
    if ($self->{'flip'}) {
        $x *= -1;
        $y *= -1;
    }
    my $dx = $x - $self->{'cur_x'};
    my $dy = $y - $self->{'cur_y'};
    my $dist = sqrt ($dx * $dx + $dy * $dy);
    my $delay = int ($dist * 1000.0 / $self->{'draw_speed'});
    $self->{'cur_x'} = $x;
    $self->{'cur_y'} = $y;
    my ($e, $m) = $self->_do_cmd("sm,$delay,$dy,$dx");
    $delay *= .95;
    usleep($delay * 1000);
    return $e;
}

# Utility routines - used by the class but not exported
#
# _do_cmd - send a command string to the egg_bot and collect
# the response. Returns the response.
sub _do_cmd {
    my ($self, $cmd, $sync) = @_;
    my $resp = "";
    my $fh = $self->{'fh'};
    my $verbose = $self->{'debug'};
    my $output;
    my $timeout = 50;
    my $result = "";

    print ":$cmd -> " if $verbose;
    syswrite ($fh, "$cmd\r");
    usleep (100_000); # give it a bit to start responding
    while ($timeout) {
        my $c;
        my $cnt = sysread($fh, $c, 255);
        if ($cnt) {
            $resp .= $c;
            last if ($resp =~ /\n/);
        } else {
            $timeout--;
            usleep (100_000);
        }
    }
    my $error = 0;
    chomp($resp);       # get rid of newline (if present)
    $resp =~ s/\r//g;   # get rid of trailing <CR> if present
    if ($timeout == 0) {
        # fake an error message if we timed out
        $resp = "!0 Err:Timeout talking to board.";
        $error = 1;
    } elsif ($sync) {
        # on sync we're looking for the version banner
        $error = $resp !~ /^EBBv13_and_above/;
    } else {
        # otherwise message other than 'OK' is an error
        $error = $resp !~ /^OK$/;
    }
    print "$resp:\n" if $verbose;
    return ($error, $resp);
}

# _setup_fd - this basically uses the POSIX termios
# class to setup the serial port to be in 'raw' mode,
# which is to say unbuffered, and without echo or translation
# on the part of the OS.  
sub _setup_fd {
    my ($fd) = @_;

    my $ti = POSIX::Termios->new() or die "No TERMIOS support!";
    $ti->setiflag( &POSIX::IGNBRK | &POSIX::IGNPAR);
    $ti->setoflag( 0 );
    $ti->setcflag( &POSIX::CS8 | &POSIX::CREAD | &POSIX::CLOCAL );
    $ti->setcc(&POSIX::VMIN, 1);
    #$ti->setcc(&POSIX::VTIME, 10);
    $ti->setispeed(&POSIX::B9600);
    $ti->setospeed(&POSIX::B9600);
    $ti->setattr($fd, &POSIX::TCSANOW);
};

1;

