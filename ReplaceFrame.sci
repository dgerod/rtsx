// ReplaceFrame.sci  replace a frame in frame structure
// www.controlsytemslab.com  August 2012
// Create a frame using Frame() first
// Example:
// F = Frame(trotx(pi/2)*transl([1 0 0])),'name','{W}');
// Fs = ReplaceFrame(Fs, F, 2) // replace frame at position 2 in the chain

function Fs=ReplaceFrame(Fs, F, fidx)
    nframes = size(Fs,1);  // number of frames in structure

    if fidx<1 | fidx>nframes then
        msg=sprintf("Frame replace location must be between 1 - %d",nframes);
        error(msg);
    end

     Fs(fidx) = F;  // replace old frame at fidx with new frame                 
      
endfunction

function Fs=replaceframe(Fs, F, fidx)
    Fs=ReplaceFrame(Fs, F, fidx);
endfunction