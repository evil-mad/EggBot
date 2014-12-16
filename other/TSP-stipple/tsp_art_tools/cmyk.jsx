// 7 October 2010

// Simple Photoshop CS5 script (JavaScript) to output in four separate files,
// the C, M, Y, and K color channels.  Each output file is output as a grayscale
// image in Photoshop (.psd) format.  The original document is left untouched.
//
// The output files will be named x-black.psd, x-cyan.psd, x-magenta.psd, and
// x-yellow.psd where "x" is the name of the original document (less its
// file extension).

function zeroAllBut(doc, name)
{
     if (doc == undefined)
     {
	  throw "No document specified"
     }
     if (name == null)
     {
	  throw "No channel name specified"
     }

     // Save the list of active channels
     var channels = doc.activeChannels

     // Loop over all the channels, zapping the ones which are not
     // the channel we want to preserve

     for (var j = 0; j < channels.length; j++)
     {
	  // Skip the "CMYK channel
	  // Skip the channel we want to preserve
	  if (channels[j].name == name || channels[j].name == "CMYK")
	  {
	       continue
	  }

	  // Zero out channel #j
	  // We do this by making channel #j the only channel active and
	  // then adjusting the output levels to (255, 255)

	  for (var i = 0; i < channels.length; i++)
	  {
	       channel = channels[i]
	       if (channel.name == channels[j].name)
	       {
		    // Make this the only channel
		    channel.visible = true
		    doc.activeChannels = new Array(channel)
	       }
	  }

	  doc.activeLayer.adjustLevels(0, 255, 1.00, 255, 255)

	  // Restore the list of channels
	  doc.activeChannels = channels
     }
}

// Access the current document
var doc = app.activeDocument

// Get the file path to the current document less it's type extension
var fname = doc.name.replace(/\..*?$/, '')
var fpath = doc.path + '/' + fname

// PSD file save options object (for later)
var psdOptions = new PhotoshopSaveOptions()

// CYAN
var c_doc = doc.duplicate(fname + "-cyan.psd")
c_doc.changeMode(ChangeMode.CMYK)
zeroAllBut(c_doc, "Cyan")
c_doc.changeMode(ChangeMode.GRAYSCALE)
c_doc.saveAs(new File(fpath + "-cyan.psd"), psdOptions)
c_doc.close(SaveOptions.DONOTSAVECHANGES)

// MAGENTA
var m_doc = doc.duplicate(fname + "-magenta.psd")
m_doc.changeMode(ChangeMode.CMYK)
zeroAllBut(m_doc, "Magenta")
m_doc.changeMode(ChangeMode.GRAYSCALE)
m_doc.saveAs(new File(fpath + "-magenta.psd"), psdOptions)
m_doc.close(SaveOptions.DONOTSAVECHANGES)

// YELLOW
var y_doc = doc.duplicate(fname + "-yellow.psd")
y_doc.changeMode(ChangeMode.CMYK)
zeroAllBut(y_doc, "Yellow")
y_doc.changeMode(ChangeMode.GRAYSCALE)
y_doc.saveAs(new File(fpath + "-yellow.psd"), psdOptions)
y_doc.close(SaveOptions.DONOTSAVECHANGES)

// BLACK (K)
var k_doc = doc.duplicate(fname + "-black.psd")
k_doc.changeMode(ChangeMode.CMYK)
zeroAllBut(k_doc, "Black")
k_doc.changeMode(ChangeMode.GRAYSCALE)
k_doc.saveAs(new File(fpath + "-black.psd"), psdOptions)
k_doc.close(SaveOptions.DONOTSAVECHANGES)
