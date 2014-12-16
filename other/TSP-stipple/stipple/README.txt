5 February 2011

stipple.py is an *unsupported* Python program to convert "point" files
produced by Adrian Secord's *unsupported* weighted Vornoi stippler to
SVG files suitable for plotting with an Eggbot.

To generated a stippled SVG file, you will need to perform the
following steps:

0. Obtain a copy of Adrian Secord's weighted Vornoi stippler from

     http://mrl.nyu.edu/~ajsecord/stipples.html

1. Find an image you wish to stipple.  Optionally,

   a. If the image's background is not already all white, then consider
      editing the image first to remove the background.  Otherwise, you
      will have the background reproduced with stipples.

   b. Rotate the image 90 degrees clockwise or counter clockwise.  This
      will be helpful later, when plotting.  Why is this useful?  The
      output from the Vornoi stippler tends to run out the stipples in
      horizontal rows, one atop the next.  When plotting this on the
      Eggbot, that generates lots of back and forth egg rotations.  It's
      better to have lots of back and forth movements of the pen arm
      with less egg rotation.  So, if you rotate the image now, stipple
      it, and then rotate it back, the resulting Eggbot plot will have
      the stipples in vertical columns stacked horizontally.  That means
      less egg rotation and thus better plotting precision.

2. Run the weighted Vornoi stippler.

   a. Open the image file with the stippler's File > Open menu item.

   b. After you open an image file, it may take a while before any stippled
      image appears. It sometimes helps to nudge things along by selecting
      "Relax Points" or "Jitter" under the "Points" menu item.

   c. You can control the number of stipples with the "Set num..." item
      under the "Points" menu.

   d. The program may run for quite some time while seeming to not be doing
      anything. It's actually trying to refine the picture it has drawn.
      You can go ahead and tell it to save its work while it is working on
      its refinements.

   e. When you have a stippling you like, use the "Save Points" item under
      the "File" menu.  This will write a file containing (x,y) coordinates
      of each point and their radii. The stipple.py script will read this
      file.

   f. If you start seeing blue stipples, that means that you're allowing
      more stipples than needed. You can decrease the number of allowed
      stipples.

   g. On OS X, when you exit the stippler it tends to exit with an error.
      You can ignore the error.

3. Run stipple.py on the file produced in Step 2e above.  From a terminal
   window, enter the command

     python stipple.py IN-FILENAME OUT-FILENAME

   where "IN-FILENAME" is the name of the file you wrote in Step 2e and
   "OUT-FILENAME" is the name of the SVG file you wish to produce.  You
   can omit specifying "OUT-FILENAME" in which case the output file will
   be the input file name with its file extension replaced with ".svg".

   For example, to process the supplied torus.pts input file file, enter
   the command

     python stipple.py torus.pts

   and the output file

     torus.svg

   will be produced.  In fact, if you use the extension ".pts" for your
   "point" files from the Vornoi stippler, then you can omit the ".pts"
   as well,

     python stipple.py torus

   On Windows systems, running Python is not as straightforward as on
   Macs and Linux systems.  See

     http://wiki.evilmadscience.com/Generating_TSP_art_from_a_stippled_image#Notes_for_Windows_users

   assistance.

4. Open the SVG file in Inkscape

   a. If you rotated the image in Step 1b, then rotate it 90 degrees in
      the opposite direction.  I.e., if you rotated the image 90 degrees
      counter clockwise, now rotate it 90 degrees clockwise.  To do this,
      in Inkscape use Edit > Select All and then Object > Rotate 90 CW.

   b. If plotting on a non-spherical object such as an egg, consider
      stretching the drawing horizontally.  Again, select all the stipples
      with Edit > Select All.  Then use Object > Transform.  In the
      Transform "subwindow" on the right, select the "Scale" tab. Then,

        i. Uncheck the "Scale proportionally" checkbox,
       ii. Uncheck the "Apply to each object separately" checkbox,
      iii. Set the units to percentages ("%"),
       iv. Enter 150% for the width,
        v. Enter 100% for the height, and
       vi. Click the "Apply" button.

5. You're now ready to plot with the Eggbot Control extension.

-- finit --
