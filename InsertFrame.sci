// InsertFrame.sci  insert a frame to frame structure
// www.controlsytemslab.com  August 2012
// Create a frame using Frame() first
// Example:
// F = Frame(trotx(pi/2)*transl([1 0 0])),'name','{W}');
// Fs = InsertFrame(Fs, F, 2) // insert at position 2 in the chain

function Fs=InsertFrame(Fs, F, fidx)
    nframes = size(Fs,1);  // number of frames in structure

    if fidx<1 | fidx>nframes+1 then
        msg=sprintf("Frame insert location must be between 1 - %d",nframes+1);
        error(msg);
    end
    if F==[] then
        F=Frame([]);        // create an unknown frame
        
    end
    if argn(2)<3 then        // append on top
        fidx = nframes+1;
        Fs(fidx) = F;
    else
        for i=nframes:-1:fidx  // push frames above fidx upwards
             Fs(i+1) = Fs(i);
         end
         Fs(fidx) = F;  // replace old frame at fidx with new frame                 
    end    
endfunction

function Fs=insertframe(Fs, F, fidx)
    Fs=InsertFrame(Fs, F, fidx);
endfunction