    //------------------------------------------Link-Class-----------------------------------------------------------//
    /*	%LINK Robot manipulator Link class
	%
	% A Link object holds all information related to a robot link such as
	% kinematics parameters, rigid-body inertial parameters, motor and
	% transmission parameters.
	%
	% Methods::
	%  A             link transform matrix
	%  RP            joint type: 'R' or 'P'
	%  friction      friction force
	%  nofriction    Link object with friction parameters set to zero
	%  dyn           display link dynamic parameters
	%  islimit       test if joint exceeds soft limit
	%  isrevolute    test if joint is revolute
	%  isprismatic   test if joint is prismatic
	%  display       print the link parameters in human readable form
	%  char          convert to string
	%
	% Properties (read/write)::
	%
	%  theta    kinematic: joint angle
	%  d        kinematic: link offset
	%  a        kinematic: link length
	%  alpha    kinematic: link twist
	%  sigma    kinematic: 0 if revolute, 1 if prismatic
	%  mdh      kinematic: 0 if standard D&H, else 1
	%  offset   kinematic: joint variable offset
	%  qlim     kinematic: joint variable limits [min max]
	%
	%  m        dynamic: link mass
	%  r        dynamic: link COG wrt link coordinate frame 3x1
	%  I        dynamic: link inertia matrix, symmetric 3x3, about link COG.
	%  B        dynamic: link viscous friction (motor referred)
	%  Tc       dynamic: link Coulomb friction
	%
	%  G        actuator: gear ratio
	%  Jm       actuator: motor inertia (motor referred)

	% - It is an error to specify 'theta' and 'd'
    % - The link inertia matrix (3x3) is symmetric and can be specified by giving
    %   a 3x3 matrix, the diagonal elements [Ixx Iyy Izz], or the moments and products
    %   of inertia [Ixx Iyy Izz Ixy Iyz Ixz].
    % - All friction quantities are referenced to the motor not the load.
    % - Gear ratio is used only to convert motor referenced quantities such as
    %   friction and interia to the link frame.
	% Notes::
	        % - Link object is a reference object, a subclass of Handle object.
	        % - Link objects can be used in vectors and arrays.
	        % - The parameter D is unused in a revolute joint, it is simply a placeholder
	        %   in the vector and the value given is ignored.
	        % - The parameter THETA is unused in a prismatic joint, it is simply a placeholder
	        %   in the vector and the value given is ignored.
	        % - The joint offset is a constant added to the joint angle variable before
	        %   forward kinematics and subtracted after inverse kinematics.  It is useful
	        %   if you  want the robot to adopt a 'sensible' pose for zero joint angle
	        %   configuration.
	        % - The link dynamic (inertial and motor) parameters are all set to
	        %   zero.  These must be set by explicitly assigning the object
	        %   properties: m, r, I, Jm, B, Tc, G.

    */
	/*Javascript class accept the same parameters of the Link class in Matlab. Works in every number of arguments.
	The most common are the four Denavit-Hartenberg kinematic parameters 
		N = new Link(theta,d,a,alpha);
	for example:
		N = new Link(math.PI/2,0,100,0);

	The rest are dynamic parameters:
		N = new Link(theta,d,a,alpha,sigma, m, rx,ry,rz, Ixx,Iyy,Izz,Ixy,Ixz,Iyz, Jm,G,B,Tc);
		N = new Link(0,1,2,3,          4,   5,  6,7,8,   9,10,11,12,13,14       , 15,16,17,18);
	*/


	function Link(){
		if (arguments.length==0){

			this.theta	=	0;		//kinematic: joint angle
	   		this.d	=	0;			//kinematic: link offset
	   		this.a	=	0;			//kinematic: link length
	   		this.alpha	=	0;		//kinematic: link twist
   			this.sigma	=	0;		//kinematic: 0 if revolute, 1 if prismatic

			this.mdh	=	0;		//kinematic: 0 if standard D&H, else 1
			this.offset	=	0;		//kinematic: joint variable offset
			this.name	=	'';		//joint coordinate name

			this.m	=	[];			//dynamic: link mass
			this.r	=	new THREE.Vector3();			//dynamic: link COG wrt link coordinate frame 3x1
			this.I	=	new THREE.Matrix3(0,0,0,0,0,0,0,0,0);			//dynamic: link inertia matrix, symmetric 3x3, about link COG.
			this.B	=	[];			//dynamic: link viscous friction (motor referred)
			this.Tc	=	[0,0];		//dynamic: link Coulomb friction

			this.G	=	0;			//actuator: gear ratio
			this.Jm	=	0; 			//actuator: motor inertia (motor referred)
			this.qlim	=	[];		//kinematic: joint variable limits [min max]
			//console.log('No arguments');
		} else if (arguments.length ==1 && arguments[0] instanceof Link){
			//return arguments[0];
			this.copy(arguments[0]);
			console.log('instanceof');
		} else {
			var opt=new LinkOptions();
			var args=arguments;
			if (args.length ==0){
				/////////// Introducir options
				/*
            		% L = Link(OPTIONS) is a link object with the kinematic and dynamic
            		% parameters specified by the key/value pairs.*/
				if (opt.theta.length!=0){

				}
			}
			//Check if arguments are numbers
			if (ArrayNum(arguments)){
	            if (arguments.length < 4){
	                console.error('must provide params (theta d a alpha)');
	            }else{
	                    
		            this.theta = arguments[0];
		            this.d = arguments[1];
		            this.a = arguments[2];
		            this.alpha = arguments[3];
		            
		            this.sigma = 0;
		            this.offset = 0;
		            this.mdh = 0;  // default to standard D&H

		            if (arguments.length >= 5){
		            	this.sigma = arguments[4];
		            }
		            if (arguments.length == 6){
	                    this.offset = arguments[5];
	                }
	                if (arguments.length > 6){
	                	//Introducing dynamyc parameters
	                	this.sigma = arguments[4];
		            	this.offset = 0;
		            	this.mdh = 0;  // default to standard D&H
						
						this.m=arguments[5];
						this.r = new THREE.Vector3(arguments[6],arguments[7],arguments[8]);
						//Inertia matrix (Ixx,Iyy,Izz,Ixy,Ixz,Iyz)
						this.I = new THREE.Matrix3(arguments[9],arguments[12],arguments[13],arguments[12],arguments[10],arguments[14],arguments[13],arguments[14],arguments[11]);
	                	
	                	if (arguments.length > 14){
	                        this.Jm = arguments[15];
	                    }
	                    if (arguments.length > 15){
	                        this.G = arguments[16];
	                    }else{
	                        this.G = 1;
	                    }
	                    if (arguments.length > 16){
	                        this.B = arguments[17];
	                    }else{
	                        this.B = 0.0;
	                    }
	                    if (arguments.length > 18){
	                        this.Tc = [arguments[18],arguments[19]];
	                    }else{
	                        this.Tc = [0,0];
	                    }
	                    this.qlim = [];
	                }else{
	                	//we know nothing about the dynamics
	                	this.m = [];
	                    this.r = new THREE.Vector3();
	                    this.I = new THREE.Matrix3(0,0,0,0,0,0,0,0,0);
	                    this.Jm = [];
	                    tis.G = 0;
	                    this.B = 0;
	                    this.Tc = [0, 0];
	                    this.qlim = [];
	                }
        		}
        	}else{
        		console.log('Link parameters are not numeric ')
        	}

            if (opt.convention == 'modified'){
                this.mdh = 1;
            }
            else{
            	this.mdh = 0;
            }
		}

	}
	
	Link.prototype.friction = function(qd){
        /*%Link.friction Joint friction force
        %
        % F = L.friction(QD) is the joint friction force/torque for link velocity QD.
        %
        % Notes::
        % - Friction values are referred to the motor, not the load.
        % - Viscous friction is scaled up by G^2.
        % - Coulomb friction is scaled up by G.
        % - The sign of the gear ratio is used to determine the appropriate
        %   Coulomb friction value in the non-symmetric case.*/
        var tau=[];
        for (var i =0; i < qd.length ; i++) {
        	var aux=this.B * Math.abs(this.G) * qd[i];
        	aux=-Math.abs(this.G) * aux;
        	tau.push(aux);
        };

        /*if issym(l)
            tau = tau + l.Tc;
        elseif qd > 0
            tau = tau + l.Tc(1);
        elseif qd < 0
            tau = tau + l.Tc(2);
        end
        */

        return tau;
    }

    Link.prototype.nofriction = function (only){
        /*
        %Link.nofriction Remove friction
        %
        % LN = L.nofriction() is a link object with the same parameters as L except
        % nonlinear (Coulomb) friction parameter is zero.
        %
        % LN = L.nofriction('all') as above except that viscous and Coulomb friction
        % are set to zero.
        %
        % LN = L.nofriction('coulomb') as above except that Coulomb friction is set to zero.
        %
        % LN = L.nofriction('viscous') as above except that viscous friction is set to zero.
        */
        var l2 = new Link(this);
        
        if (arguments.length == 0){
            only = 'coulomb';
        }
        
        switch (only){
            case 'all':
                l2.B = 0;
                l2.Tc = [0, 0];
            case 'viscous':
                l2.B = 0;
            case 'coulomb':
                l2.Tc = [0,0];
        }
        return l2;
    }

    Link.prototype.RP = function(){
        /*%Link.RP Joint type
        %
        % c = L.RP() is a character 'R' or 'P' depending on whether joint is revolute or
        % prismatic respectively.
        % If L is a vector of Link objects return a string of characters in joint order.
        */
        var v='';
        if (this.sigma == 0){
        	v = 'R';
        }else{
            v = 'P';
        }
        return v;
    }

    Link.prototype.setr = function(v){
    	/*
        %Link.r Set centre of gravity
        %
        % L.r = R set the link centre of gravity (COG) to R (3-vector).
        %*/
        if ( v instanceof THREE.Vector3){
        	this.r=v;
        } else if (v instanceof Array){
        	if (v.length == 3 && ArrayNum(v)){
	            this.r = new THREE.Vector3(v[0],v[1],v[2]);
	        }else{
	        	console.error('RTB:Link:badarg', 'COG must be a 3-vector');
	        }
        }
    }

	Link.prototype.setTc =function (v){
		/*
        %Link.Tc Set Coulomb friction
        %
        % L.Tc = F set Coulomb friction parameters to [F -F], for a symmetric
        % Coulomb friction model.
        %
        % L.Tc = [FP FM] set Coulomb friction to [FP FM], for an asymmetric
        % Coulomb friction model. FP>0 and FM<0.
       	*/
       	if (!isNaN(v)){
       		this.Tc = [v, -v]; 
       	}else if (v instanceof Array){
        	if ( v.length <= 2 && ArrayNum(v) ){
        		if (v.length == 1 ){
        			this.Tc = [v[0], -v[0]]; 
        		}
        		if (v.length == 2 && v[0]<v[1]){
        			this.Tc = [v[0], v[1]]; 
        		}else{
        			console.error('RTB:Link:badarg', 'Coulomb friction is [Tc+ Tc-], therefore Tc+ must be greater than Tc-. Tc+ > Tc-');
        		}
        	}else{
        		console.error('Coulomb friction vector can have 1 (symmetric) or 2 (asymmetric) elements only');
        	}
        }
        /*if isa(v,'sym') && ~isempty(findsym(v))
            l.Tc = sym('Tc');
        elseif isa(v,'sym') && isempty(findsym(v))
            v = double(v);
        end
        */
    }

    Link.prototype.setI = function (M){
    	/*
        %Link.I Set link inertia
        %
        % L.I = [Ixx Iyy Izz] set link inertia to a diagonal matrix.
        %
        % L.I = [Ixx Iyy Izz Ixy Iyz Ixz] set link inertia to a symmetric matrix with
        % specified inertia and product of intertia elements.
        %
        % L.I = M set Link inertia matrix to M (3x3) which must be symmetric.
        */

        if ( M instanceof THREE.Matrix3){
        	var M_Aux=new THREE.Matrix3();
        	var G_Aux=new THREE.Matrix3();
        	M_Aux.copy(M);
        	G_Aux.copy(M);
        	G_Aux.transpose();
        	if(compareArray(M_Aux.elements,G_Aux.elements)){
        		this.I.copy(M);
        	}else{
        		console.error('inertia matrix must be symmetric');
        	}
        } else if (M instanceof Array){
        	//Matrix definition by diagonal elements
        	if (M.length == 3 && ArrayNum(M)){
        		this.I = new THREE.Matrix3(M[0],0,0,		0,M[1],0,		0,0,M[2]);
        	}else if (M.length == 6 && ArrayNum(M)){
           		//Matrix definition by I=[Ixx,Iyy,Izz,Ixy,Ixz,Iyz]
        		this.I = new THREE.Matrix3(M[0],M[3],M[4],M[3],M[1],M[5],M[4],M[5],M[2]); 
        	}else{
        		console.error('RTB:Link:badarg', 'must set I to 3-vector, 6-vector');
        	}
        } else{
                console.error('RTB:Link:badarg', 'must set I to 3-vector, 6-vector or symmetric 3x3');
        }
    }

    Link.prototype.isLimit = function (q){
    	/*
        %Link.islimit  Test joint limits
        %
        % L.islimit(Q) is true (1) if Q is outside the soft limits set for this joint.
        */

        if (!ArrayNum(this.qlim)){
            console.log('no limits assigned to link');
        }else{
        	if (this.qlim[0]<=this.qlim[1]){
        		return ((q > this.qlim[0] ) && (q < this.qlim[1]));
        	}else{
        		console.error('RTB:Link:badarg, Minimum qlim angle is greater than the maximum. Limit angles are [qlim_min qlim_max]');
        	}
        }
    }

	Link.prototype.isrevolute= function(){
		return (this.sigma == 0);
	}

	Link.prototype.isprismatic= function(){
        return (this.sigma == 1);
    }

    Link.prototype.A = function (q){
    	/*
        %Link.A Link transform matrix
        %
        % T = L.A(Q) is the link homogeneous transformation matrix (4x4) corresponding
        % to the link variable Q which is either the Denavit-Hartenberg parameter THETA (revolute)
        % or D (prismatic).
        %
        % Notes::
        % - For a revolute joint the THETA parameter of the link is ignored, and Q used instead.
        % - For a prismatic joint the D parameter of the link is ignored, and Q used instead.
        % - The link offset parameter is added to Q before computation of the transformation matrix.*/
        
        if (!isNaN(q)){
        	var T =new THREE.Matrix4();
            var sa = Math.sin(this.alpha); 
            var ca = Math.cos(this.alpha);
            q = q + this.offset;
            if (this.sigma == 0){
                // revolute
                var st = Math.sin(q);
                var ct = Math.cos(q);
                var d = this.d;
            }else {
                // prismatic
                var st = Math.sin(this.theta); 
                var ct = Math.cos(this.theta);
                var d = q;
            }
            
            if (this.mdh == 0){
                // standard DH
                
                T =new THREE.Matrix4(ct , -st*ca , st*sa , this.a*ct , st , ct*ca , -ct*sa , this.a*st , 0 , sa , ca , d,0,0,0,1);
            }else{
                // modified DH
                
                T =new THREE.Matrix4(ct , -st , 0 , this.a , st*ca , ct*ca , -sa, -sa*d , st*sa , ct*sa , ca ,ca*d ,0,0,0,1);
            } 
            return T;
        	
        }   else{
        	console.error('RTB:Link:badarg, Parameter is not a numeric number');
        }
    }

    Link.prototype.copy = function (copyLink){
    	var newLink = new Link();
    	if (copyLink instanceof Link){
			for(var key in copyLink) {
				if (typeof(copyLink[key])!='function'){
					if (copyLink[key] instanceof THREE.Vector3){
						eval('this.'+key+ ' = new THREE.Vector3()');
						//console.log('this.'+key+ '.copy(copyLink.'+key+')');
						eval('this.'+key+ '.copy(copyLink.'+key+')');
						//console.log ('Parameter '+key+ ' is a Vector3');
					}else if  (copyLink[key] instanceof THREE.Matrix3){
						eval('this.'+key+ ' = new THREE.Matrix3()');
						eval('this.'+key+ '.copy(copyLink.'+key+')');
						//console.log ('Parameter '+key+ ' is a Matrix3');
					}else if  (copyLink[key] instanceof THREE.Matrix4){
						eval('this.'+key+ ' = new THREE.Matrix4()');
						eval('this.'+key+ '.copy(copyLink.'+key+')');
						//console.log ('Parameter '+key+ ' is a Matrix4');
					}else if  (copyLink[key] instanceof Array){
						eval('this.'+key+ ' = ['+copyLink[key]+']');
						//console.log ('Parameter '+key+ ' is an Array');
					} else if (key=='name'){
                        this['name']='Link';
                    }else{
						//alert('newLink.'+key+ ' = '+copyLink[key]);
						eval('this.'+key+ ' = '+copyLink[key]);
					}

				}else{
					//console.log(key);
				}
			}
    	}
    }


    //------------------------------------------SerialLink-Class-----------------------------------------------------------//
    /*
    %SerialLink Create a SerialLink robot object
    %
    % R = SerialLink(LINKS, OPTIONS) is a robot object defined by a vector 
    % of Link objects.
    %
    % R = SerialLink(DH, OPTIONS) is a robot object with kinematics defined
    % by the matrix DH which has one row per joint and each row is
    % [theta d a alpha] and joints are assumed revolute.  An optional 
    % fifth column sigma indicate revolute (sigma=0, default) or 
    % prismatic (sigma=1).
    %
    % R = SerialLink(OPTIONS) is a null robot object with no links.
    %
    % R = SerialLink([R1 R2 ...], OPTIONS) concatenate robots, the base of
    % R2 is attached to the tip of R1.
    %
    % R = SerialLink(R1, options) is a deep copy of the robot object R1, 
    % with all the same properties.
    %
    % Options::
    %
    %  'name', name            set robot name property
    %  'comment', comment      set robot comment property
    %  'manufacturer', manuf   set robot manufacturer property
    %  'base', base            set base transformation matrix property
    %  'tool', tool            set tool transformation matrix property
    %  'gravity', g            set gravity vector property
    %  'plotopt', po           set plotting options property
    %

        % Robot objects can be concatenated in two ways
        %      R = R1 * R2;
        %      R = SerialLink([R1 R2]);
        %
        % Note::
        % - SerialLink is a reference object, a subclass of Handle object.
        % - SerialLink objects can be used in vectors and arrays
        % - When robots are concatenated (either syntax) the intermediate base and
        %   tool transforms are removed since general constant transforms cannot 
        %   be represented in Denavit-Hartenberg notation.
        %
    */
	function SerialLink(){
		this.name  =  '';			//name of robot, used for graphical display
		this.links  =  [];			//vector of Link objects (1xN)
		this.n=0;					//number of joints
		this.comment  =  '';		//annotation, general comment
		this.manuf='';				//annotation, manufacturer's name

        this.gravity= new THREE.Vector3(0, 0, 9.81);				//direction of gravity [gx gy gz]
        this.base= new THREE.Matrix4().identity();				//pose of robot's base (4x4 homog xform)
        this.tool=new THREE.Matrix4().identity();				//robot's tool transform, T6 to tool tip (4x4 homog xform)

        this.qteach=0;
        this.mdh=0;					// kinematic convention boolean (0=DH, 1=MDH)
        this.fast_rne =0;
       	this.qlim  	=	[];			//joint limits, [qmin qmax] (Nx2)
		this.offset =	0;			//kinematic joint coordinate offsets (Nx1)

        this.plotopt=0;
        this.lineopt=0;
        this.shadowopt=0;


		if (arguments.length==0){
			//Si no hay parametros se crea un SerialLInk vacio

		}else if (arguments.length ==1 && arguments[0] instanceof SerialLink){
			//return arguments[0];
			console.log('Instance of SerialLins');
		}else if(arguments[0] instanceof Array){
			try{
				for (var i =0; i < arguments[0].length ; i++) {
					if (!arguments[0][i] instanceof Link) throw "Invalid argument. No Link class";
					var aux_Link=new Link();
					aux_Link.copy(arguments[0][i]);
					this.links.push(aux_Link);
				};
				console.log('Instance Of Array Links');
			}catch (err) {
				alert('Warning');
				console.log(err);
			}
		} else {
			try{
				for (var i =0; i < arguments.length ; i++) {
					if (!arguments[i] instanceof Link) throw "Invalid argument. No Link class";
					var aux_Link=new Link();
					aux_Link.copy(arguments[i]);
					this.links.push(aux_Link);
				};
				console.log('Instance Of Links');
			}catch (err) {
				alert('Warning');
				console.log(err);
			}
		}

		var opt=new SerialLinkOptions();
		var args=arguments;
		//FALTAN ESTAS PARTES//
		// copy the properties to robot object
		// set the robot object mdh status flag


		//-------------------------SERIALLINK - METHODS-----------------------//

		SerialLink.prototype.setTool = function(M){
			this.tool = new THREE.Matrix4().identity();
            if (M==null){
                this.tool.identity();
            } else if (!isHomog(M) || !(M instanceof THREE.Matrix4)){
                console.error('SerialLink:badarg. Tool must be a homogeneous transform');
            }else{
            	if (M instanceof THREE.Matrix4){
            	    this.tool.copy(M);
            	}
            }
        }

        SerialLink.prototype.setBase = function (M){
            this.base = new THREE.Matrix4().identity();
            if (M==null){
                this.base.identity();
            } else if (!isHomog(M) || !(M instanceof THREE.Matrix4)){
                console.error('SerialLink:badarg. Base must be a homogeneous transform');
            }else{
            	if (M instanceof THREE.Matrix4){
	                this.base.copy(M);
	            }
            }
        }

        SerialLink.prototype.setOffset = function(v){
            if(v instanceof Array){
                if (v.length == this.links.length){
                    for (var i = 0; i < this.links.length; i++) {
                        this.links[i].offset=v[i];
                    }
                }else{
                    console.error('SerialLink:badarg. Argument for setOffset. Vector length must equal number DOF');
                }
            }else{
                console.error('SerialLink:badarg. Invalid parameter. Array required for setOffset');
            }
        }

        SerialLink.prototype.setQlim = function(v){
            if (v instanceof Array) {
                if (v.length == this.links.length){
                    if (subArrayIn(v)){
                        var aux=true;
                        //Array [  [qlim11 , qlim12] , [qlim21, qlim22], [qlim31,qlim32] ...];
                        for (var i = 0; i < this.links.length; i++) {
                            if (v[i].length ==2){
                                if (v[i][0] <= v[i][1]){
                                    //this.links[i].qlim= [v[i][0],v[i][1]];
                                }else{
                                    aux=false;
                                    console.error('SerialLink:badarg. qlim[1] must be lower than qlim[2] for setQlim');
                                }
                            }else{
                                console.error('SerialLink:badarg. Invalid Subarray length for setQlim');
                                aux=false;
                            }
                        }
                        if (aux){
                            this.qlim=v;
                            for (var i = 0; i < this.links.length; i++) {
                                this.links[i].qlim= [v[i][0],v[i][1]];
                            }
                        }
                    }else{
                        console.error('SerialLink:badarg. Invalid parameter. Subarray for setQlim  method is wrong length');
                    }
                } else if (v.length == 2*this.links.length && ArrayNum(v)){
                    var aux=true;
                    //Array [qlim11 , qlim12 , qlim21, qlim22, ...];
                    for (var i = 0; i < this.links.length; i++) {
                        if (v[2*i] <= v[2*i+1]){

                        }else{
                            aux=false;
                            console.error('qlim[1] must be lower than qlim[2]');
                        }
                    }
                    if (aux){
                        for (var i = 0; i < this.links.length; i++) {
                            this.links[i].qlim = [v[2*i],v[2*i+1]];
                            this.qlim.push([v[2*i],v[2*i+1]]);
                        }
                    }
                } else {
                    console.error('SerialLink:badarg. Argument for setQlim  method is wrong length');
                }
            } else{
                console.error('SerialLink:badarg. Invalid parameter. Array required for setQlim');
            }
        }

        SerialLink.prototype.getQlim = function (){
            var qlim = [];
            for (var i = 0; i < this.links.length; i++) {
                if (this.links[i].isrevolute()){
                    qlim.push([this.links[i].qlim[0],this.links[i].qlim[1]]);
                } else if (this.links[i].isprismatic()){
                    qlim.push([0,0]);
                }else{
                    console.error ('SerialLink:badarg. Something is going wrong. Nor prismatic nor revolute joint');
                }
            };
            return qlim;
        }

        SerialLink.prototype.setGravity = function (v){
            if (v instanceof THREE.Vector3){
                this.gravity.copy(v);
            }else{
                console.error('SerialLink:badarg. Gravity must be a 3-vector');
            }
        }

        SerialLink.prototype.isLimit = function(q){
        /*
        %SerialLink.islimit Joint limit test
        %
        % V = R.islimit(Q) is a vector of boolean values, one per joint, 
        % false (0) if Q(i) is within the joint limits, else true (1).
        %
        % Notes::
        % - Joint limits are purely advisory and are not used in any
        %   other function.  Just seemed like a useful thing to include.*/
            if (q instanceof Array){
                if (q.length == this.links.length && ArrayNum(q)){
                    var aux = [];
                    for (var i = 0; i < this.links.length; i++) {
                        aux[i]=this.links[i].isLimit(q[i]);
                    };
                    return aux;
                }else{
                    console.error('SerialLink:badarg. Argument for isLimit method is wrong length or not numeric');
                }
            }else{
                console.error('SerialLink:badarg. Invalid parameter. Array required for isLimit');
            }
        }

        SerialLink.prototype.isspherical = function (){
        /*
        %SerialLink.isspherical Test for spherical wrist
        %
        % R.isspherical() is true if the robot has a spherical wrist, that is, the 
        % last 3 axes are revolute and their axes intersect at a point.
        %
        % See also SerialLink.ikine6s.*/
            var aux=false;
            var zeros=false;
            n=this.links.length;
            if (this.links[n-3].a==0 && this.links[n-2].a==0 && this.links[n-1].a==0 && this.links[n-2].d==0 && this.links[n-1].d==0){
                zeros = true;
            }
            if (Math.abs(this.links[n-3].alpha)==Math.PI/2 && (Math.abs(this.links[n-3].alpha + this.links[n-2].alpha)< Math.EPSILON)){
                aux=true;
            }
        }

        SerialLink.prototype.payload = function(m, p){
        /*
        %SerialLink.payload Add payload mass
        %
        % R.payload(M, P) adds a payload with point mass M at position P 
        % in the end-effector coordinate frame.
        %
        % See also SerialLink.rne, SerialLink.gravload.*/
            if (!isNan (m) || (p instanceof THREE.Vector3)){
                var n=this.links.length;
                this.links[n].m;
                this.links[n].r.copy(p);
            }else {
                console.error('SerialLink.payload :badarg. Invalid parameter. Vector3 required for mass position or numeric mass');        
            }
        }

        SerialLink.prototype.rne = function(a1, a2, a3, a4, a5){

            var z0 = [0,0,1];
            var grav = this.gravity;   //% default gravity from the object
            var fext = [0, 0, 0, 0, 0, 0];
            var n = this.links.length;

            //% check that robot object has dynamic parameters for each link
            for (var i = 0; i < n; i++) {
                var r_aux= new THREE.Vector3();
                var m_aux= new THREE.Matrix3(0,0,0,0,0,0,0,0,0);
                if (this.links[i].r.equals(r_aux) || this.links[i].m ==0 || compareArray(this.links[i].I.elements,m_aux.elements)){
                    console.error('SerialLink.rne :badarg. Dynamic parameters (m, r, I) not set in link '+i);
                }
            }
            /*% Set debug to:
            %   0 no messages
            %   1 display results of forward and backward recursions
            %   2 display print R and p**/
            var debug = 0;
            var Q=[];
            var QD=[];
            var QDD=[];
            // a1 = [ [q1,q2,q3...,qd1,qd2,qd3....,qdd1,qdd2,qdd3], [], []]

            //Constructor Function
            if (subArrayIn(arguments[0]) && !subArrayIn(arguments[1]) && !subArrayIn(arguments[1]) ){
                var ncol=arguments[0][0].length;

                if (ncol==3*n){
                    for (var i = 0; i < n; i++) {
                        Q.push(arguments[0][i].slice(0,n));
                        QD.push(arguments[0][i].slice(n,2*n));
                        QDD.push(arguments[0][i].slice(2*n,3*n));
                    };

                    if (arguments.length==2){
                        if (arguments[1] instanceof THREE.Vector3){
                            var grav=new THREE.Vector3();
                            grav.copy(arguments[1]);
                        }else{
                            console.error('SerialLink.rne :badarg. Invalid parameter. Vector3 required for gravity ');      
                        }
                    }
                    if (arguments.length==3){
                        if (ArrayNum(arguments[2])  && arguments[2].length==6){
                            var fext=arguments[2].slice();
                            console.log(fext);
                        }else{
                            console.error('SerialLink.rne :badarg. Invalid parameter. Array [Fx,Fy,Fz,Mx,My,Mz] required for External Payload');         
                        }
                    }
                }else{
                    console.error('SerialLink.rne :badarg. Invalid parameter. First array length != 3*nlinks');         
                }
            }else if (subArrayIn(arguments[0]) ||  subArrayIn(arguments[1]) || subArrayIn(arguments[2])){
                if (arguments[0][0].length==n || arguments[1][0].length==n || arguments[2][0].length==n){
                        Q=arguments[0].slice();
                        QD=arguments[1].slice();
                        QDD=arguments[2].slice();
                }else{
                    console.error('SerialLink.rne :badarg. Invalid parameter. Argument wrong length (length = nlinks)');         
                }
                if (arguments.length==4){
                    if (arguments[3] instanceof THREE.Vector3){
                        var grav=new THREE.Vector3();
                        grav.copy(arguments[3]);
                    }else{
                        console.error('SerialLink.rne :badarg. Invalid parameter. Vector3 required for gravity ');      
                    }
                }
                if (arguments.length==5){
                    if (ArrayNum(arguments[4])  && arguments[2].length==6){
                        var fext=arguments[4].slice();
                        console.log(fext);
                    }else{
                        console.error('SerialLink.rne :badarg. Invalid parameter. Array [Fx,Fy,Fz,Mx,My,Mz] required for External Payload');         
                    }
                }
            }
            /*console.log(Q);
            console.log(QD);
            console.log(QDD);
            */
            
            /*if isa(Q, 'sym')
                tau(np, n) = sym();
            else
                tau = zeros(np,n);
            end*/
            var nrows = Q.length;
            var q,qd,qdd,Fm,Nm,Rm,w,wd,vd,pstar,alpha;
            for (var i = 0; i < nrows; i++) {
                q=Q[i].slice();
                qd=QD[i].slice();
                qdd=QDD[i].slice();
                //console.log(q);

                Fm=[];
                Nm=[];
                pstarm=[];
                Rm=[];
                w=new THREE.Vector3();
                wd=new THREE.Vector3();
                vd=Grav.clone();

               for (var j = 0; j < n ; j++) {
                    var link = new Link();
                    link.copy(this.links[j]);
                    console.log(link);
                    var Tj = new THREE.Matrix4();
                    Tj=link.A(q[j]);
                    //console.log(Tj);
                    if (link.RP() == 'R'){
                        d = link.d;
                    }
                    else if (link.RP() == 'P'){
                        d = q(j);
                    }else{
                        console.error('SerialLink.rne :ERROR. Link.RP not defined. Revolute or Prismatic');
                    }
                    alpha = link.alpha;
                    pstar = [link.a, d*Math.sin(alpha), d*Math.cos(alpha)];
                };
            };
            /*
            % init some variables, compute the link rotation matrices
            %*/
                /*for j=1:n
                    link = robot.links(j);
                    Tj = link.A(q(j));
                    if link.RP == 'R'
                        d = link.d;
                    else
                        d = q(j);
                    end
                    alpha = link.alpha;
                    pstar = [link.a; d*sin(alpha); d*cos(alpha)];
                    if j == 1
                        pstar = t2r(robot.base) * pstar;
                        Tj = robot.base * Tj;
                    end
                    pstarm(:,j) = pstar;
                    Rm{j} = t2r(Tj);
                    if debug>1
                        Rm{j}
                        Pstarm(:,j).'
                    end
                end

            /*%
            %  the forward recursion
            %*/
        }

    }

	SerialLink.prototype.links	=	[]; //vector of Link objects (1xN)
	SerialLink.prototype.gravity	=9.81;    //direction of gravity [gx gy gz]
	SerialLink.prototype.base  	=	0;       //pose of robot's base (4x4 homog xform)
	SerialLink.prototype.tool  	=	0;      //robot's tool transform, T6 to tool tip (4x4 homog xform)
	SerialLink.prototype.qlim  	=	[];   //joint limits, [qmin qmax] (Nx2)
	SerialLink.prototype.offset =	0;      //kinematic joint coordinate offsets (Nx1)
	SerialLink.prototype.manuf 	=	0;       //annotation, manufacturer's name
	SerialLink.prototype.plotopt=	0;     //options for plot() method (cell array)

	SerialLink.prototype.addLink  = function(theta,d,a,alpha,sigma){
		this.links.push(new Link(theta,d,a,alpha,sigma));
	}

	function LinkOptions(){
		LinkOptions.prototype.theta = [];
		LinkOptions.prototype.a = 0;
		LinkOptions.prototype.d = [];
		LinkOptions.prototype.alpha = 0;
		LinkOptions.prototype.G = 0;
		LinkOptions.prototype.B = 0;
		LinkOptions.prototype.Tc = [0, 0];
		LinkOptions.prototype.Jm = [];
		LinkOptions.prototype.I = new THREE.Matrix3(0,0,0,0,0,0,0,0,0);
		LinkOptions.prototype.m = [];
		LinkOptions.prototype.r = new THREE.Vector3();
		LinkOptions.prototype.offset = 0;
		LinkOptions.prototype.qlim = [];
		LinkOptions.prototype.type = ['revolute', 'prismatic', 'fixed'];
		LinkOptions.prototype.convention = ['standard', 'modified'];
		LinkOptions.prototype.sym = false;
	}

	function SerialLinkOptions(){
		LinkOptions.prototype.name = 'robot';
		LinkOptions.prototype.comment = [];
		LinkOptions.prototype.manufacturer = [];
		LinkOptions.prototype.base = new THREE.Matrix4().identity();
		LinkOptions.prototype.tool = new THREE.Matrix4().identity();
		LinkOptions.prototype.offset = [];
		LinkOptions.prototype.qlim = [];
		LinkOptions.prototype.plotopt = [];
	}

	function ArrayNum(array){
    		var result=true;
    		for (var i = 0; i < array.length; i++) {
    			if (typeof array[i] != 'number'){
    			//if(isNaN(array[i])){
    				//alert( array[i]+' '+typeof array[i])
    				result=false;
    			}
    		}
		return result;
	}

    function subArrayIn(array){
        var result=true;
        if (array instanceof Array){
            for (var i = 0; i < array.length; i++) {
                var n=array[0].length;
                if (!array[i] instanceof Array || array[i].length !=n || !ArrayNum(array[i])){
                    result = false;
                }
            };
        } else{
            result = false;
        }
        return result;
    }

	function isSymmetric(M,N,grade){
		if((M instanceof THREE.Matrix3 || M instanceof THREE.Matrix4 ) && (N instanceof THREE.Matrix3 || N instanceof THREE.Matrix4 )){
			if (M.length != N.length) { 
				return false; 
			}else{
				if (M.length ==3){
					var M_Aux=new THREE.Matrix3();
					var N_Aux=new THREE.Matrix3();
				}
				if (M.length ==4){
					var M_Aux=new THREE.Matrix4();
					var N_Aux=new THREE.Matrix4();
				}

				M_Aux.copy(M);
				N_Aux.copy(N);
				return(compareArray(M_Aux.elements,N_Aux.elements));
			}
		}
	}

	function compareArray(arrayA, arrayB) {
        if (arrayA.length != arrayB.length) { return false; }
        var a=arrayA;
        var b=arrayB;
        //a.sort(); 
        //b.sort();

        for (var i = 0, l = a.length; i < l; i++) {
            if (a[i] !== b[i]) { 
                return false;
            }
        }
        return true;
    }

    function isHomog(M){
    	// Comprueba si la matriz es homogenea
    	return true;
    }
