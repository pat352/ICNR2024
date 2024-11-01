classdef Filter < handle
    
    
    properties (Access=public)
        % Filter properties
        type='';
        order=0;
        Fc=0;
        Ts=0;
        % Low pass coefs
        k4=0;
        k3=0;
        k2=0;
        k1=0;
        k0=0;
        MAX_ORDER=6;
        
        % High pass coef
        j0=0;
        j1=0;
        j2=0;
        % array
        y=ones(1,6);
        u=ones(1,6);
        % output
        out;
    end
    
    properties (Constant, Access=public)
        SQRT2 = sqrt(2.0);
        SQRT3 = sqrt(3.0);
        SQRT5 = sqrt(5.0);
    end
    
    methods (Access=public)
        
        % Properties of the fitler
        function obj=Filter(type,order,Fc,Ts)
            obj.type=type;
            obj.order=order;
            obj.Fc=Fc;
            obj.Ts=Ts;
            
            if strcmp(obj.type,'HighPass')
                obj.initHighPass();
            end
            if strcmp(obj.type,'LowPass')
                obj.initLowPass();
            end
        end
        
        % Initiate filter
        function initLowPass(obj)            
            switch obj.order
                case 0
                    % nothing
                    
                case 1
                    a  = 2.0*pi*obj.Fc;
                    obj.k1 = exp(a*obj.Ts);
                    obj.k0 = 1.0 - obj.k1;
                    
                case 2
                    a  = -pi*obj.Fc*obj.SQRT2;
                    b  =  pi*obj.Fc*obj.SQRT2;
                    obj.k2 = exp(2.0*obj.Ts*a);
                    obj.k1 = 2.0*exp(a*obj.Ts)*cos(b*obj.Ts);
                    obj.k0 = 1.0 - obj.k1 + obj.k2;
                    
                case 3
                    a  = -pi*obj.Fc;
                    b  =  pi*obj.Fc*obj.SQRT3;
                    c  =  2.0*pi*obj.Fc;
                    b3 = exp(-c*obj.Ts);
                    b2 = exp(2.0*obj.Ts*a);
                    b1 = 2.0*exp(a*obj.Ts)*cos(b*obj.Ts);
                    obj.k3 = b2*b3;
                    obj.k2 = b2 + b1*b3;
                    obj.k1 = b1 + b3;
                    obj.k0 = 1.0 - b1 + b2 -b3 + b1*b3 - b2*b3;
                    
                case 4
                    a  = -0.3827*2.0*pi*obj.Fc;
                    b  =  0.9238*2.0*pi*obj.Fc;
                    c  = -0.9238*2.0*pi*obj.Fc;
                    d  =  0.3827*2.0*pi*obj.Fc;
                    b4 = exp(2.0*obj.Ts*c);
                    b3 = 2.0*exp(c*obj.Ts)*cos(d*obj.Ts);
                    b2 = exp(2.0*obj.Ts*a);
                    b1 = 2.0*exp(a*obj.Ts)*cos(b*obj.Ts);
                    % Coeficients
                    obj.k4 = b2*b4;
                    obj.k3 = b1*b4 + b2*b3;
                    obj.k2 = b4 + b1*b3 + b2;
                    obj.k1 = b1 + b3;
                    obj.k0 = 1.0 - obj.k1 + obj.k2 - obj.k3 + obj.k4;
            end

        end
        function obj=initHighPass(obj)
            k=2.0/obj.Ts;
            w0=2.0*pi*obj.Fc;
            
            switch obj.order
                case 1
                    b0=k;
                    b1=(-1)*k;
                    a0=(w0+k);
                    a1=(w0-k);
                    % Differential equation terms
                    obj.j0=b0/a0;
                    obj.j1=b1/a0;
                    obj.k1=(-1)*a1/a0;
                    
                case 2
                    w0sq=w0^2;
                    ksq=k^2;
                    % TF terms
                    b0=ksq;
                    b1=(-2)*ksq;
                    b2=ksq;
                    a0=w0sq+k*w0+ksq;
                    a1=2*w0sq-2*ksq;
                    a2=w0sq-k*w0+ksq;
                    % Diff equation terms
                    obj.j0=b0/a0;
                    obj.j1=b1/a0;
                    obj.j2=b2/a0;
                    obj.k1=(-1)*a1/a0;
                    obj.k2=(-1)*a2/a0;
                    
                case 3
                    w0sq=w0^2;
                    ksq=k^2;
                    % TF terms
                    b0=ksq;
                    b1=(-2)*ksq;
                    b2=ksq;
                    a0=w0sq+k*w0+ksq;
                    a1=2*w0sq-2*ksq;
                    a2=w0sq-k*w0+ksq;
                    % Diff equation terms
                    obj.j0=b0/a0;
                    obj.j1=b1/a0;
                    obj.j2=b2/a0;
                    obj.k1=(-1)*a1/a0;
                    obj.k2=(-1)*a2/a0;
                    
                case 4
                    w0sq=w0^2;
                    ksq=k^2;
                    % TF terms
                    b0=ksq;
                    b1=(-2)*ksq;
                    b2=ksq;
                    a0=w0sq+k*w0+ksq;
                    a1=2*w0sq-2*ksq;
                    a2=w0sq-k*w0+ksq;
                    % Diff equation terms
                    obj.j0=b0/a0;
                    obj.j1=b1/a0;
                    obj.j2=b2/a0;
                    obj.k1=(-1)*a1/a0;
                    obj.k2=(-1)*a2/a0;
   
            end
        end
        function inData(obj, input)
            if strcmp(obj.type,'HighPass')
                obj.ComputeHighPass(input);
            end
            if strcmp(obj.type,'LowPass')
                obj.ComputeLowPass(input);
            end 
        end
        function ComputeLowPass(obj, input)
            % Rotate data
            for i=obj.MAX_ORDER:-1:2
                obj.y(i) = obj.y(i-1);
                obj.u(i) = obj.u(i-1);
            end
            obj.u(1)=input;
            
            % Calculate 
            switch obj.order
                case 0
                    obj.y(1)=input;
                case 1
                    obj.y(1) = obj.k1*obj.y(2) + obj.k0*input;
                case 2
                    obj.y(1) = obj.k1*obj.y(2) - obj.k2*obj.y(3) + (obj.k0*input);
                case 3
                    obj.y(1) = obj.k1*obj.y(2) - obj.k2*obj.y(3) + obj.k3*obj.y(4) + (obj.k0*input);
                case 4
                    obj.y(1) = obj.k1*obj.y(2) - obj.k2*obj.y(3) + obj.k3*obj.y(4) - obj.k4*obj.y(5) + (obj.k0*input);
            end
            % Return the calculated output
            obj.out=obj.y(1);
        end
        function ComputeHighPass(obj,input)
            % Rotate data
            for i=obj.MAX_ORDER:-1:2
                obj.y(i) = obj.y(i-1);
                obj.u(i) = obj.u(i-1);
            end
            obj.u(1)=input;
            switch obj.order
                case 0
                    obj.y(1)=input;
                case 1
                    obj.y(1)=obj.k1*obj.y(2)+obj.j0*obj.u(1)+obj.j1*obj.u(2);
                case 2
                    obj.y(1)=obj.k1*obj.y(2)+obj.k2*obj.y(3)+obj.j0*obj.u(1)+obj.j1*obj.u(2)+obj.j2*obj.u(3);
                case 3
                    obj.y(1)=obj.k1*obj.y(2)+obj.k2*obj.y(3)+obj.j0*obj.u(1)+obj.j1*obj.u(2)+obj.j2*obj.u(3);
                case 4
                    obj.y(1)=obj.k1*obj.y(2)+obj.k2*obj.y(3)+obj.j0*obj.u(1)+obj.j1*obj.u(2)+obj.j2*obj.u(3);    
            end
            % Return the calculated output
            obj.out=obj.y(1);
        end
    end
    
    
end