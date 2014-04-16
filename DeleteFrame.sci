// DeleteFrame.sci  delete a frame in frame structure
// www.controlsytemslab.com  August 2012

// Example:
// Fs = DeleteFrame(Fs, 2) // delete at position 2 in the chain

function Fs=DeleteFrame(Fs,fidx)
    nframes = size(Fs,1);  // number of frames in structure

    if fidx<1 | fidx>nframes then
        msg=sprintf("Frame delete location must be between 1 - %d",nframes);
        error(msg);
    end
     for i=fidx:nframes-1  // shrink frames above fidx downwards
         Fs(i) = Fs(i+1);
     end
     Fs(nframes) = [];  // delete the excess 
endfunction

function Fs=deleteframe(Fs, fidx)
    Fs=DeleteFrame(Fs, fidx);
endfunction