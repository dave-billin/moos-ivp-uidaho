function DataBytes = ToSerialBytes( SourceData, ShouldSwapBytes )
%TOSERIALBYTES Converts fixed-point data to an array of Bytes
% Converts fixed-point data elements to a uint8 array of data Bytes
%
% PARAM: SourceData
%   Fixed-point data (int/uint8 ... int/uint64) to be serialized to data
%   Bytes
%
% PARAM: ShouldSwapBytes
%   (optional) true if the Byte order of SourceValue should be swapped
%   during the conversion; else false to leave Byte order unmodified
%
% RETURNS:
%   If SourceValue is a scalar, a row vector of uint8 elements 
%   corresponding to the Bytes of SourceValue is returned.  If SourceValue 
%   is an MxN matrix, an MxN cell array is returned, whose cells contain
%   vectors of Bytes corresponding to the elements of SourceValue.
%   Elements in Byte vectors produced by this function are ordered such
%   that element(1) = Byte 0, element(2) = Byte 1, etc.
%
ShouldSwap = false;


    % Should we swap Byte order?
    if (nargin > 1)
        assert( isa(ShouldSwapBytes, 'logical'), ...
                'Parameter ShouldSwapBytes must be a logical value');
        ShouldSwap = ShouldSwapBytes;
        
        if (ShouldSwap)
            SourceData = swapbytes(SourceData);
        end
    end
    
    %-------------------------------
    % Convert to Bytes
    %-------------------------------
    if ( isscalar(SourceData) )     % Case: input is a scalar
        DataBytes = typecast(SourceData, 'uint8');
        
    else    % Case: input is a matrix
        [M, N] = size(SourceData);
        DataBytes = cell([M, N]);
        
        for i = 1 : M
            for j = 1 : N
                DataBytes{i, j} = typecast(SourceData(i,j), 'uint8');
            end
        end
    end

end
