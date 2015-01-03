classdef AHRSMessage < handle
    %IMUMESSAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        RollAngle           % Measured inertial roll angle in degrees
        PitchAngle          % Measured inertial pitch angle in degrees
        YawAngle            % Measured inertial yaw angle in degrees
        
        RollRate            % Measured rate of roll change (degrees/sec)
        PitchRate           % Measured rate of pitch change (degrees/sec)
        YawRate             % Measured rate of yaw change (degrees/sec)

    end
    
    methods
    % Public methods
    
        function obj = AHRSMessage(varargin)
        % IMUMESSAGE Creates an AHRSMessage object
        % All properties are initialized to zero
            zero = single(0);
            obj.RollAngle = zero;
            obj.PitchAngle = zero;
            obj.YawAngle = zero;

            obj.RollRate = zero;
            obj.PitchRate = zero;
            obj.YawRate = zero;
        end
        
        
        function DataBytes = Serialize(obj)
        % SERIALIZE Serializes the IMU message to a row vector of Bytes
            Signature = uint8( ones(1,6) * 255); % Signature Bytes
            MessageID = uint8( hex2dec('55') ); % Message identifier
            
            Angle = [ typecast(obj.RollAngle, 'uint8'), ...
                      typecast(obj.PitchAngle, 'uint8'), ...
                      typecast(obj.YawAngle, 'uint8') ];
                       
            AngleRate = [ typecast(obj.RollRate, 'uint8'), ...
                          typecast(obj.PitchRate, 'uint8'), ...
                          typecast(obj.YawRate, 'uint8') ];
                     
            Pad = uint8( zeros(1,35) );
            
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
            DataBytes = [ Signature, MessageID, Angle, AngleRate, ...
                          Pad, CRC ];
        end

        
        %----------------------------
        % Property set/get methods
        %----------------------------
        
        % RollAngle
        function set.RollAngle(obj, Angle)
            assert( isscalar(Angle), ...
                    'RollAngle must be a scalar value' )
            obj.RollAngle = single(Angle);
        end
        
        % PitchAngle
        function set.PitchAngle(obj, Angle)
            assert( isscalar(Angle), ...
                    'PitchAngle must be a scalar value' )
            obj.PitchAngle = single(Angle);
        end

        % YawAngle
        function set.YawAngle(obj, Angle)
            assert( isscalar(Angle), ...
                    'YawAngle must be a scalar value' )
            obj.YawAngle = single(Angle);
        end
        
        % RollRate
        function set.RollRate(obj, Rate)
            assert( isscalar(Rate), ...
                    'RollRate must be a scalar value' )
            obj.RollRate = single(Rate);
        end
        
        % PitchRate
        function set.PitchRate(obj, Rate)
            assert( isscalar(Rate), ...
                    'PitchRate must be a scalar value' )
            obj.PitchRate = single(Rate);
        end
        
        % YawRate
        function set.YawRate(obj, Rate)
            assert( isscalar(Rate), ...
                    'YawRate must be a scalar value' )
            obj.YawRate = single(Rate);
        end
    end
    
end

