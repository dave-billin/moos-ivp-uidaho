classdef IMUMessage < handle
    %IMUMESSAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        GyroStatus = [];        % Logical row vector of gyro status as
                                % [ GyroX1, GyroY1, GyroZ1 ] where 0=pass
                                % and 1=fail
                                
        Accelerometer1Status = []; % Logical row vector of acclelerometer
                                   % status [ AccX1, AccY1, AccZ1 ] where
                                   % 0=pass and 1=fail

        Accelerometer2Status = []; % Logical row vector of acclelerometer
                                   % status [ AccX2, AccY2, AccZ2 ] where
                                   % 0=pass and 1=fail

        IsAligning              % true if the IMU is running its alignment 
                                % process; else false
                                
        IsFastErecting          % true if the IMU is running its fast-
                                % alignment process; else false
                                
        IsWarmBootData          % true if data is the first sample after a
                                % warm boot; else false
                                
        InternalFaultIsDetected % true if the IMU has detected an internal
                                % fault condition; else false
                                
        DataRate = 0;           % 8-bit value marked as TBD in docs
        
        DeltaRollAngle          % Change in roll angle (degrees/sample)
        DeltaPitchAngle         % Change in pitch angle (degrees/sample)
        DeltaYawAngle           % Change in yaw angle (degrees/sample)

        DeltaVLongitudinal      % Measured longitudinal velocity
        DeltaVLateral           % Measured lateral velocity
        DeltaVNormal            % Measured normal velocity
        
        DeltaBodyVelocity = []; % Change in body velocity since last sample
                                % as [ longitudinal, lateral, normal ]
    end
    
    methods
    % Public methods
    
        function obj = IMUMessage(varargin)
        % IMUMESSAGE Creates an IMUMessage object
        % All properties are initialized to zero
            Zero1by3 = single(zeros(1,3));
            obj.GyroStatus = Zero1by3;
            obj.Accelerometer1Status = Zero1by3;
            obj.Accelerometer2Status = Zero1by3;
            obj.IsAligning = false;
            obj.IsFastErecting = false;
            obj.IsWarmBootData = false;
            obj.InternalFaultIsDetected = false;
            obj.DataRate = uint8(0);
            
            obj.DeltaRollAngle = single(0);
            obj.DeltaPitchAngle = single(0);
            obj.DeltaYawAngle = single(0);
            
            obj.DeltaVLongitudinal = single(0);
            obj.DeltaVLateral = single(0);
            obj.DeltaVNormal = single(0);
            
            obj.DeltaBodyVelocity = Zero1by3;

        end
        
        
        function DataBytes = Serialize(obj)
        % SERIALIZE Serializes the IMU message to a row vector of Bytes
            Signature = uint8( ones(1,6) * 255); % Signature Bytes
            MessageID = uint8( hex2dec('AA') ); % Message identifier
            
            % Construct the first status Byte
            StatusWord = [0 0 0 0];
            Accum = 0;
            for i = 1 : 3
                Accum = Accum + obj.GyroStatus(i) * 2^(i-1);
                Accum = Accum + obj.Accelerometer1Status(i) * 2^(i+2);
                Accum = Accum + obj.Accelerometer2Status(i) * 2^(i+5);
            end
            StatusWord(1) = Accum;
            
            % Construct the second status Byte
            Accum = 0;
            if (obj.IsAligning), Accum = Accum + 1; end
            if (obj.IsFastErecting), Accum = Accum + 2; end
            if (obj.IsWarmBootData), Accum = Accum + 4; end
            if (obj.InternalFaultIsDetected), Accum = Accum + 8; end
            StatusWord(2) = Accum;
            
            Drate = typecast(obj.DataRate, 'uint8');
            
            DeltaAngle = [ typecast(obj.DeltaRollAngle, 'uint8'), ...
                           typecast(obj.DeltaPitchAngle, 'uint8'), ...
                           typecast(obj.DeltaYawAngle, 'uint8') ];
                       
            DeltaVel = [ typecast(obj.DeltaVLongitudinal, 'uint8'), ...
                         typecast(obj.DeltaVLateral, 'uint8'), ...
                         typecast(obj.DeltaVNormal, 'uint8') ];
                     
            Pad = uint8( zeros(1,30) );
            
%             % Create a CRC-16 CRC generator, then use it to generate
%             % a checksum for the
%             % binary vector represented by the ASCII sequence '123456789'.
%             gen = crc.generator('Polynomial', '0x04C11DB7', ...
%             'ReflectInput', true, 'ReflectRemainder', true);
%                         
%             msg = reshape(de2bi(49:57, 8, 'left-msb')', 72, 1);
%             encoded = generate(gen, msg);

            CRC = uint8( zeros(1,4) );
            
            % Serialize the data Bytes
            DataBytes = [ Signature, MessageID, StatusWord, Drate, ...
                          DeltaAngle, DeltaVel, Pad, CRC ];
        end

        
        %----------------------------
        % Property set/get methods
        %----------------------------
        
        % GyroStatus
        function set.GyroStatus(obj, Status)
            assert( isequal( size(Status), [1,3] ), ...
                    'GyroStatus must be a 1x3 vector');
            obj.GyroStatus = Status;
        end
        
        % Accelerometer1Status
        function set.Accelerometer1Status(obj, Status)
            assert( isequal( size(Status), [1,3] ), ...
                    'Accelerometer1Status must be a 1x3 vector');
            obj.Accelerometer1Status = Status;
        end
        
        % Accelerometer2Status
        function set.Accelerometer2Status(obj, Status)
            assert( isequal( size(Status), [1,3] ), ...
                    'Accelerometer2Status must be a 1x3 vector');
            obj.Accelerometer2Status = Status;
        end
        
        % IsAligning
        function set.IsAligning(obj, State)
            obj.IsAligning = (State ~= 0);
        end
        
        % IsFastErecting
        function set.IsFastErecting(obj, State)
            obj.IsFastErecting = (State ~= 0);
        end
        
        % IsWarmBootData
        function set.IsWarmBootData(obj, State)
            obj.IsWarmBootData = (State ~= 0);
        end
        
        % InternalFaultIsDetected
        function set.InternalFaultIsDetected(obj, State)
            obj.InternalFaultIsDetected = (State ~= 0);
        end
        
        % DeltaRollAngle
        function set.DeltaRollAngle(obj, Angle)
            assert( isscalar(Angle), ...
                    'DeltaRollAngle must be a scalar value' );
            obj.DeltaRollAngle = single(Angle);
        end
        
        % DeltaPitchAngle
        function set.DeltaPitchAngle(obj, Angle)
            assert( isscalar(Angle), ...
                    'DeltaPitchAngle must be a scalar value' );
            obj.DeltaPitchAngle = single(Angle);
        end
        
        % DeltaYawAngle
        function set.DeltaYawAngle(obj, Angle)
            assert( isscalar(Angle), ...
                    'DeltaYawAngle must be a scalar value' );
            obj.DeltaYawAngle = single(Angle);
        end
        
        % DeltaVLongitudinal
        function set.DeltaVLongitudinal(obj, DeltaV)
            assert( isscalar(DeltaV), ...
                    'DeltaVLongitudinal must be a 1x3 row vector' );
            obj.DeltaVLongitudinal = single(DeltaV);
        end

        % DeltaVLateral
        function set.DeltaVLateral(obj, DeltaV)
            assert( isscalar(DeltaV), ...
                    'DeltaVLateral must be a 1x3 row vector' );
            obj.DeltaVLateral = single(DeltaV);
        end
        
        % DeltaVNormal
        function set.DeltaVNormal(obj, DeltaV)
            assert( isscalar(DeltaV), ...
                    'DeltaVNormal must be a 1x3 row vector' );
            obj.DeltaVNormal = single(DeltaV);
        end
        
        % DataRate
        function set.DataRate(obj, Rate)
            obj.DataRate = uint8(Rate);
        end

    end
    
end

