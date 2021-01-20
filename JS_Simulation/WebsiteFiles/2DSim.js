'use strict';
/**
 * This file defines the simulation implementation.
 * The simulation is defined in the class Simulation2D, where
 * each physical object is a subclass of SimObject, combining drawing and physics.
 */
const b2Vec2 = Box2D.Common.Math.b2Vec2;
const Dynamics = Box2D.Dynamics;
const b2_dynamicBody = Dynamics.b2Body.b2_dynamicBody;

function addVec(a,b){ return a.map((v,i)=>v+b[i]); }
function subVec(a,b){ return a.map((v,i)=>v-b[i]); }
function rotate2dVector(vec, angle){
	const co=Math.cos(angle), si=Math.sin(angle);
	return [co*vec[0]-si*vec[1],si*vec[0]+co*vec[1]]
}

/**
 * Baseclass for all physical components in the simulation.
 * Assumes rectangle shape and handles drawing, physics and coordinate transforms.
 */
class SimObject {
	constructor(world, position, angle, dimensions, color, ground, body_type=b2_dynamicBody, use20x20Friction=false){
		this.BOX2D_SCALE_FACTOR = 300; // U=300, aka 300 pixels is a meter
		this.w = dimensions[0],	this.h = dimensions[1], this.color=color;
		var phyW = this.w/this.BOX2D_SCALE_FACTOR, phyH = this.h/this.BOX2D_SCALE_FACTOR;
		// define physical Body
		var phyBody = new Dynamics.b2BodyDef();
		phyBody.position.Set(position[0]/this.BOX2D_SCALE_FACTOR, position[1]/this.BOX2D_SCALE_FACTOR);
		phyBody.angle=angle;
		phyBody.linearDamping = phyBody.angularDamping = 0.0;
		phyBody.type=body_type;
		this.phyBody = world.CreateBody(phyBody);

		var phyFix = new Dynamics.b2FixtureDef();
		phyFix.shape = new Box2D.Collision.Shapes.b2PolygonShape();
		phyFix.shape.SetAsBox(phyW/2, phyH/2);
		phyFix.density = 1, phyFix.friction = 0.3, phyFix.restitution = 0.5;
		this.phyBody.CreateFixture(phyFix);
		// enable friction if necessary
		if(ground != undefined && body_type==b2_dynamicBody){
			var fric = new Dynamics.Joints.b2FrictionJointDef();
			fric.bodyA = this.phyBody;
			fric.bodyB = ground;
			if (use20x20Friction) phyH = phyW = 20/this.BOX2D_SCALE_FACTOR; // forceSmaller
			fric.maxForce = phyW*phyH*10;
			fric.maxTorque = fric.maxForce * Math.sqrt(phyW*phyW+phyH*phyH)/3*0.3;
			world.CreateJoint(fric);
		}
	}
	getPos(array=false){
		var returner = this.phyBody.GetPosition().Copy();
		returner.Multiply(this.BOX2D_SCALE_FACTOR);
		if(array){ return [returner.x, returner.y];	}
		return returner;
	}
	getAngle(){return this.phyBody.GetAngle();}
	getWorldPoint(localPoint){
		localPoint.Multiply(1/this.BOX2D_SCALE_FACTOR)
		var returner = this.phyBody.GetWorldPoint(localPoint);
		returner.Multiply(this.BOX2D_SCALE_FACTOR)
		return returner;
	}
	getLocalPose(worldPoint, angle){
		worldPoint.Multiply(1/this.BOX2D_SCALE_FACTOR)
		var returner = this.phyBody.GetLocalPoint(worldPoint);
		returner.Multiply(this.BOX2D_SCALE_FACTOR)
		return [returner.x, returner.y, angle-this.getAngle()]
	}
	getWorldVector(x, y){
		return this.phyBody.GetWorldVector(new b2Vec2(x, y))
	}
	setVelocity(velocity){ // sets the velocity in Box2D. velocity: [v_x, v_y, v_rot] in [pixel/s, pixel/s, degrees]
		this.phyBody.SetLinearVelocity(new b2Vec2(velocity[0]/this.BOX2D_SCALE_FACTOR, velocity[1]/this.BOX2D_SCALE_FACTOR))
		this.phyBody.SetAngularVelocity(velocity[2]*Math.PI/180);
	}
	draw(){
		const pos = this.getPos();
		drawRect(pos.x, pos.y, this.w, this.h, this.color, this.getAngle());
	}
	update(dt){}
	returnToView(){
		var pos = this.phyBody.GetPosition()
		const 	min = 25.0/this.BOX2D_SCALE_FACTOR,
				maxX = (canvas.width-25)/this.BOX2D_SCALE_FACTOR,
				maxY = (canvas.height-25)/this.BOX2D_SCALE_FACTOR;
		var change=false;
		if(pos.x < min){
			pos.x = min;	change=true;
		} else if(pos.x>maxX){
			pos.x = maxX; 	change=true;
		} 
		if(pos.y < min){
			pos.y = min;	change=true;
		} else if(pos.y>maxY){
			pos.y = maxY; 	change=true;
		} 
		if(change) this.phyBody.SetPosition(pos);
	}
}

/** A single gripper finger with closing motor */
class Finger extends SimObject {
	constructor(world, gripper, rightFinger){
		var dimensions = [30,15];
		var dirFactor = rightFinger? 1 :-1;
		var maxY = dirFactor*(gripper.h-dimensions[1])/2;
		var position = gripper.getWorldPoint(new b2Vec2((gripper.w+dimensions[0])/2, maxY))
		super(world, [position.x, position.y], gripper.getAngle(), dimensions, "black");
		this.dirFactor = dirFactor;
		this.maxY = maxY;
		
		// Joint between gripper and finger
		var jointDef = new Dynamics.Joints.b2PrismaticJointDef();
		jointDef.bodyA = gripper.phyBody;
		jointDef.bodyB = this.phyBody;
		jointDef.collideConnected = false;
		jointDef.localAxisA = new b2Vec2(0,dirFactor);
		jointDef.localAnchorA = new b2Vec2((gripper.w)/2/this.BOX2D_SCALE_FACTOR,0)
		jointDef.localAnchorB = new b2Vec2(-dimensions[0]/2/this.BOX2D_SCALE_FACTOR,0)
		jointDef.enableLimit = true;
		jointDef.upperTranslation = Math.abs(maxY)/this.BOX2D_SCALE_FACTOR;
		jointDef.lowerTranslation = dimensions[1]/2/this.BOX2D_SCALE_FACTOR;
		jointDef.enableMotor = true;
		jointDef.maxMotorForce = 50;
		jointDef.motorSpeed = 0;
		this.joint = world.CreateJoint(jointDef);
	}
	setMotorSpeed(velocity){
		this.joint.SetMotorSpeed(-velocity)
	}
}

/**
 * The robot gripper to be controlled by the user.
 * This class handles user input, adaptive and standard robot control.
 * It also provides the methods to calculate gripper-centric environment states.
 */
class Gripper extends SimObject {
	constructor(world, simulation, position, angle, ground){
		var dimensions = [15,90];
		super(world, position, angle, dimensions, "red", ground, Dynamics.b2Body.b2_kinematicBody);
		this.dimensions = dimensions;
		this.CHOSEN_DOF_UPDATE_TIME = 5000; //idle time until a dof is chosen new in ms
		this.speedPerDim = [50.0, 50.0, 70.0, 0.05];
		this.simulation = simulation;
		this.adaptive_keys_origin = ['w','s','d','a']
		this.numDofSelector= document.getElementById('numDofSelect')
		this.numAdaptiveDofs = this.numDofSelector.value
		this.adaptive_keys = this.adaptive_keys_origin.slice(0,2* this.numAdaptiveDofs)
		this.keys_std = ['ArrowUp','ArrowDown','ArrowRight','ArrowLeft', '0', 'Control', '3', '1'];
		this.keys_std_2 = ['i','k','l','j','o','u','m','n'];
		this.dofs_std = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]; // one per row
		// Adaptive dofs, last should be empty initially, so that the init state is not used for dof smoothing
		this.dofs = [[0,0,1,0],[0.5/Math.sqrt(0.5),0.5/Math.sqrt(0.5),0.8,0],[0,1,0,0],[0,0,0,0]]; 
		this.eigVals = [0,0,0,0];
		this.resetChosenDofs = true;
		this.chosenDofs = Array.from(Array(this.adaptive_keys.length/2).keys());
		this.updateChosenDofs(true)
		this.velocity = [0,0,0,0];
		this.keysPressed = this.adaptive_keys.concat(this.keys_std).reduce((result,key)=>{result[key]=false;return result;},{});
		this.fingers = [new Finger(world, this, false), new Finger(world, this, true)]
		this.requestRunning = false; // weather a DoF request is running
		this.lastRequestPkg = {"model":"", "state":[0,0,0,0,0,0,0,0]};
		this.modelSelector = document.getElementById('modelSelect');
		this.changingDofSelector = document.getElementById('DofTrackingType');
		this.dofAutoSwitchSelector = document.getElementById('DofAutoSwitch');
		this.dofSignMethod = document.getElementById('DoF_SignMethod');
	}
	/* Get the state of the environment relative to the gripper as 8-dim value */
	getState(){
		var targetPos = this.getLocalPose(new b2Vec2(this.simulation.target.x, this.simulation.target.y),0);
		var boxPos = [];
		for (var box of this.simulation.boxes){
			boxPos.push(this.getLocalPose(box.getPos(), box.getAngle()));
		}
		function normaliseRotation(dif){
			dif = Math.sign(dif)*(Math.abs(dif)%(2*Math.PI));
			return Math.abs(dif)<Math.PI ? dif : dif - Math.sign(dif)*2*Math.PI;
		}
		return [ this.velocity[0], this.velocity[1], this.velocity[2], this.velocity[3],
				targetPos[0], targetPos[1],
				boxPos[0][0], boxPos[0][1], normaliseRotation(boxPos[0][2]),
				boxPos[1][0], boxPos[1][1], normaliseRotation(boxPos[1][2]),
			]
	}
	scaleVelocityVector(vector){
		var normSquared = vector[0]*vector[0] + vector[1]*vector[1]
		if (normSquared >1){
			var norm = Math.sqrt(normSquared)
			vector[0] /= norm;
			vector[1] /= norm;
		}
		return [vector[0]*this.speedPerDim[0], vector[1]*this.speedPerDim[1], 
				vector[2]*this.speedPerDim[2], vector[3]*this.speedPerDim[3]];
	}
	updateChosenDofs(ignoreTimeStamp=false){
		if(ignoreTimeStamp || Date.now()-this.chosenDof_timer > this.CHOSEN_DOF_UPDATE_TIME){ // update chosen dof once timer has run out
			this.chosenDof_timer = Date.now();
			if(this.resetChosenDofs){	// if reset is called, set to initial values
				this.resetChosenDofs = false
				const newChosenDofs = Array.from(Array(this.adaptive_keys.length/2).keys());
				if(ignoreTimeStamp || !newChosenDofs.reduce((r,v,i)=>r&=(v==this.chosenDofs[i]),true)){
					this.chosenDofs = newChosenDofs;
					return;
				}
			}
			for(var i = 0; i<this.chosenDofs.length; i++){
				var v = this.chosenDofs[i];
				while(this.chosenDofs.includes(v)){
					v = (v+1)%this.dofs.length;
				}
				this.chosenDofs[i] = v;
			}
		}
	}
	// calculate the closest chosenDofs and store new_dofs, eigvals
	storeMostSimilarChosenDofs(new_dofs, newEigVals){
		// calc distances 
		function square(c){return c*c;}
		const oldChosDistToNew = this.chosenDofs.map((v)=>{return this.dofs[v];}).map((oCd)=>{ // for each old chosen DoF
				return new_dofs.map((nCd)=>{	// check every new DoF
					const dists = [1,-1].map((sign)=>{		// for each sign of vector
						return [oCd.reduce((r,v,j)=>{return r+square(v-sign*nCd[j]) },0), sign]}); // get distance to DoF (with sign and index)
					return dists[1][0]>dists[0][0]?dists[0]:dists[1]; // return smaller distance Value
			});});		
		//check best DoF permutation
		const permuts = this.chosenDofs.length==1?[[0],[1],[2],[3]]: 
			[[0,1],[0,2],[0,3],[1,0],[1,2],[1,3],[2,0],[2,1],[2,3],[3,0],[3,1],[3,2]]
		const permutDists = permuts.map((perm)=>{return perm.reduce((r,v,i)=>{return r+oldChosDistToNew[i][v][0]},0)});
		const newChosenDofs =  permuts[permutDists.reduce((r,v,i)=>{return v<permutDists[r]?i:r},0)];

		if(this.changingDofSelector.value=="trackDofs"){
			this.chosenDofs = newChosenDofs; // store tracked DoFs
		}
		if(this.dofSignMethod.value=='keepOrientation'){ // handle vector sign
			for(var i=0; i<newChosenDofs.length;i++){ 
				new_dofs[newChosenDofs[i]] = new_dofs[newChosenDofs[i]].map((v)=>{return v*oldChosDistToNew[i][newChosenDofs[i]][1]})
			}
		} else if(this.dofSignMethod.value=='largestComponent') {
			var indexOfAbsLargest = new_dofs.map(vec=>vec.reduce((r,v,i)=>Math.abs(v)>Math.abs(vec[r])?i:r,0));
			new_dofs = new_dofs.map((vec, i)=>vec.map(v=>v*Math.sign(new_dofs[i][indexOfAbsLargest[i]])));
		}
		this.dofs = new_dofs;
		this.eigVals = newEigVals;
	}
	anyAdaptiveKeyPressed(){
		for(var key of this.adaptive_keys){
			if(this.keysPressed[key]){
				return true;
			}
		}
		if(this.joystickInput){
			for(var i= 0; i<this.numAdaptiveDofs; i++){
				if(this.joystickInput[i] != 0){ 
					return true;
				}
			}
		}
		return false;
	}
	update(dt){
		if(this.numAdaptiveDofs != this.numDofSelector.value){ // handle UI Changes of num dofs
			this.numAdaptiveDofs = this.numDofSelector.value;
			this.adaptive_keys = this.adaptive_keys_origin.slice(0,this.numAdaptiveDofs*2);
			this.resetChosenDofs = true;
			this.updateChosenDofs(true);
		}
		
		var velocity = [0,0,0,0];

		if(USE_MODEL){
			var somethingPressed = this.anyAdaptiveKeyPressed();

			if(!this.requestRunning && this.modelSelector.value){ // if we're not waiting and a model exists
				const pkg = {"model":this.modelSelector.value, "state":this.getState().slice(4)}
				if(pkg['model'] != this.lastRequestPkg['model'] || !pkg['state'].reduce((r,v,i)=>r&&(v==this.lastRequestPkg['state'][i]), true)){ // only send request if anything changed
					this.lastRequestPkg = pkg; // store currently send package
					this.requestRunning = true;
					ioSocket.emit(DOF_REQUEST, pkg, (dofs, eigVals)=>{
						if(!(this.changingDofSelector.value=='updateDofsOnlyIdle' && this.anyAdaptiveKeyPressed())){ // dont update if a key is pressed and user chose OnlyUpdateIdle
							this.storeMostSimilarChosenDofs(dofs,eigVals);
							this.requestRunning = false;
						}			
					})
				}
			}
			if(!somethingPressed){
				if(this.changingDofSelector.value=='updateDofsOnlyIdle') 
					this.requestRunning = false;
				if(this.dofAutoSwitchSelector.value=="true") 
					this.updateChosenDofs()				
			} else {
				this.chosenDof_timer = Date.now(); // remember the time of last input
				this.resetChosenDofs = true;	// if adaptive control was used, reset to default on next run
			}
			for(var i=0; i<this.adaptive_keys.length/2; i++){
				if(this.keysPressed[this.adaptive_keys[2*i]])
					velocity = addVec(velocity, this.dofs[this.chosenDofs[i]]);
				if(this.keysPressed[this.adaptive_keys[2*i+1]])
					velocity = subVec(velocity, this.dofs[this.chosenDofs[i]]);
			}
		}
		if(this.joystickInput){
			if(USE_MODEL){
				for(var i=0; i<this.numAdaptiveDofs; i++){
					velocity = velocity.map((v,j)=>{return v + this.joystickInput[i]*this.dofs[this.chosenDofs[i]][j];})
				}
			} else {
				velocity = addVec(velocity, this.joystickInput);
			}
		}
		for(var i=0; i<this.keys_std.length/2; i++){
			if(this.keysPressed[this.keys_std[2*i]] || this.keysPressed[this.keys_std_2[2*i]])
				velocity = addVec(velocity, this.dofs_std[i]);
			if(this.keysPressed[this.keys_std[2*i+1]] || this.keysPressed[this.keys_std_2[2*i+1]])
				velocity = subVec(velocity, this.dofs_std[i]);
		}
		
		this.velocity = velocity.slice(); // store user input velocity
		// transform gripper relative translations to global ones
		var newPos = this.getWorldVector(velocity[0], velocity[1]);
		velocity[0] = newPos.x;
		velocity[1] = newPos.y;
		velocity = this.scaleVelocityVector(velocity);
		
		this.setVelocity(velocity.slice(0,3))
		this.fingers.forEach(f=>f.setMotorSpeed(velocity[3]))
	}
	draw(){
		super.draw();
		this.fingers.forEach(f=>f.draw())
	}
	keyDown_specialButtons(key){
		switch(key){ // switch adaptive dofs
			case 'q':
				this.resetChosenDofs = true;
				this.updateChosenDofs(true);
				return true;
			case 'e':
				this.resetChosenDofs = false; // actively stop reseting
				this.updateChosenDofs(true);
				return true;
		}		
		return false;
	}
	keyPressed(key){
		var consumeKey = false;
		this.keysPressed[key] = true;
		consumeKey |= this.keyDown_specialButtons(key);
		for(var v of [this.adaptive_keys, this.keys_std, this.keys_std_2]){
			if(v.includes(key)){
				consumeKey = true;
				break;
			}
		}
		return consumeKey
	}
	keyReleased(key){
		this.keysPressed[key] = false;
	}
}

/** The pure visual target in the simulation. */
class Target {
	constructor(position){
		this.size = 10;
		this.x = position[0];
		this.y = position[1];
	}
	draw(){
		drawCirc(this.x, this.y, this.size, 'red')
	}
	update(dt){}
}

/** The camera class handling recording. */
class Camera {
	constructor(gripper){
		this.gripper = gripper;
		this.run = 0;
	}
	update(dt){
		if(recordingButton.value == "true"){
			ioSocket.emit(STORE_STATE, this.gripper.getState());
		} else this.run = 0;
	}
	countSimRestart(gripper){
		this.gripper = gripper;
		this.run++;
	}
	draw(){
		if(recordingButton.value == "true"){
			writeText(canvas.width, 0, `Recording run #${this.run}`, {align:'right', baseline:'top'});
		}
	}
}

/** Boxes to be manipulated. Can be toggled to have poles / spikes. */
class Box extends SimObject{
	constructor(world, position, angle, dimensions, color, ground, body_type=b2_dynamicBody, use20x20Friction=false){
		super(world, position, angle, dimensions, color, ground, body_type, use20x20Friction);
		this.togglePole(document.getElementById('poleButton').value=="true");
	}
	
	createPole(){
		this.poleWidth = 5, this.poleHeight=30;
		var phyFix = new Dynamics.b2FixtureDef();
		phyFix.shape = new Box2D.Collision.Shapes.b2PolygonShape();
		phyFix.shape.SetAsBox(this.poleHeight/this.BOX2D_SCALE_FACTOR/2, this.poleWidth/this.BOX2D_SCALE_FACTOR/2);
		this.localOffset = new Box2D.Common.Math.b2Vec2((this.w+this.poleHeight)/this.BOX2D_SCALE_FACTOR/2,0);
		phyFix.shape.m_vertices.forEach(e=>e.Add(this.localOffset));
		phyFix.density = 1, phyFix.friction = 0.3, phyFix.restitution = 0.5;
		this.poleFixture = this.phyBody.CreateFixture(phyFix);			
	}
	togglePole(setPoles){
		if(setPoles) this.createPole();
		else if(this.poleFixture != undefined){
			this.poleFixture = this.phyBody.DestroyFixture(this.poleFixture);			
		}
	}
	draw(){
		super.draw();
		if(this.poleFixture != undefined){
			const pos = this.phyBody.GetWorldPoint(this.localOffset);
			drawRect(pos.x*this.BOX2D_SCALE_FACTOR, pos.y*this.BOX2D_SCALE_FACTOR, this.poleHeight, this.poleWidth, this.color, this.getAngle());
		}
	}
}

/** 
 * The simulation environment that is being set up and run in a loop. 
 * Also handles the HUD.
 */
class Simulation2D {
	constructor(){
		this.handleJSEvents();
	}
	addWalls(world, ground){
		const wallstrength = 100; // Wall thickness. Should be large to not be able to allow tunneling
		const w = canvas.width;
		const h = canvas.height;
		var s = [new SimObject(world, [0-wallstrength/2, h/2],0, [wallstrength, h], 'white', ground, Dynamics.b2Body.b2_staticBody),
				 new SimObject(world, [w+wallstrength/2, h/2],0, [wallstrength, h], 'white', ground, Dynamics.b2Body.b2_staticBody),
				 new SimObject(world, [w/2, 0-wallstrength/2],0, [w, wallstrength], 'white', ground, Dynamics.b2Body.b2_staticBody),
				 new SimObject(world, [w/2, h+wallstrength/2],0, [w, wallstrength], 'white', ground, Dynamics.b2Body.b2_staticBody)]
	}
	getRandomPosition(){
		function getAxisValue(length){
			var offset = length/5;
			var random_area = length - 2*offset;
			return offset + Math.random()*random_area;
		}
		return [getAxisValue(canvas.width), getAxisValue(canvas.height)];
	}
	getRandomRotation(){
		return Math.random()*2*Math.PI;
	}
	restart(){
		this.setup();
		ioSocket.emit(RESTARTING_SIM);
	}
	keyDown_specialButtons(key){
		switch(key.key){
			case 'Escape':
				this.cancel();
				return true;
			case 'h':
				toggleHUD(this.hudButton);
				return true;
			case 'r':
				console.log("Restarting Simulation")
				this.restart();
				return true;
			case 'p':
				var state = this.gripper.getState();
				console.log(state)
				alert(state);
				return true;
		}
		return false;
	}
	handleJSEvents(){	
		window.addEventListener('keydown', (key) => {
			var consumeKey = this.keyDown_specialButtons(key);
			consumeKey |= this.gripper.keyPressed(key.key);
			if(consumeKey) key.preventDefault()
		});
		window.addEventListener('keyup', (key)=>this.gripper.keyReleased(key.key));

		window.addEventListener("gamepadconnected", (e)=>{
			if(navigator.userAgent.indexOf("Firefox") != -1) // dirty solution to solve the gamepad not-updating bug on edge/chrome
				this.joystick = ()=>e.gamepad
			else{
				this.joystick = ()=>navigator.getGamepads()[0]
			}
			this.joystick_oldButtons = Array.from(this.joystick().buttons, e=>false);
		});
		window.addEventListener("gamepaddisconnected", (e)=>this.joystick=undefined)
		
		window.addEventListener('blur', ()=>{ // when the window looses focus
			this.oldJoystick = this.joystick;
			this.joystick = undefined;
			Object.keys(this.gripper.keysPressed).forEach(k=> this.gripper.keysPressed[k] = false)
		})
		window.addEventListener('focus', ()=>this.joystick = this.oldJoystick) // when the window gets focus
	}
	setup(){
		this.world = new Dynamics.b2World(new b2Vec2(0,0), false)
		var ground = new Dynamics.b2BodyDef();
		ground.type = Dynamics.b2Body.b2_staticBody;
		this.ground = this.world.CreateBody(ground)		
		this.simElements = [];
		this.addWalls(this.world, this.ground);
		this.target = new Target(this.getRandomPosition());
		this.simElements.push(this.target);
		this.gripper = new Gripper(this.world, this, this.getRandomPosition(), this.getRandomRotation(), this.ground);
		this.simElements.push(this.gripper);
		this.boxes = [ 	new Box(this.world, this.getRandomPosition(), this.getRandomRotation(),[20,20], 'blue', this.ground,b2_dynamicBody, true),
						new Box(this.world, this.getRandomPosition(), this.getRandomRotation(),[30,30], 'blue', this.ground,b2_dynamicBody, true) ];
		for(var box of this.boxes)	this.simElements.push(box); 
		if(!this.camera){
			this.camera = new Camera(this.gripper);
		} else this.camera.countSimRestart(this.gripper);
		this.simElements.push(this.camera);
		this.hudButton = document.getElementById("toggleHudButton");
		this.hideDofOverlay = false;
		this.sleeping = false;
		
		this.frameCounter = 0;
		this.fps = 0;
		this.time = Date.now()
	}
	getArrowTip(lines,scale=1){
		const vec = subVec(lines.slice(-2,-1)[0], lines.slice(-1)[0]).map((v)=>{return v*scale;});
		var a =  [1,-1].map((v)=>{return addVec(lines.slice(-1)[0], rotate2dVector(vec,20*v*Math.PI/180))});
		a.push(lines.slice(-1)[0]);
		return a;
	}
	draw_dof(dof, color){
		const scaledDof = this.gripper.scaleVelocityVector(dof).
							map((v)=>{return v*0.2;})// Scale arrow length;
		// XY Rot Coordinates
		var centerPoints = [this.gripper.getPos(true)]
		var angles = [this.gripper.getAngle()]
		for(var i=0; i<10;i++){
			centerPoints.push(addVec(centerPoints[i], rotate2dVector(scaledDof.slice(0,2),angles[i])));
			angles.push(angles[i]+scaledDof[2]*Math.PI/180);
		}
		const externalPointsToDraw = [[0,this.gripper.fingers[0].maxY], [0,this.gripper.fingers[1].maxY]];
		for(var p of externalPointsToDraw){
			const newLine = centerPoints.map((v,i)=>{return addVec(v,rotate2dVector(p,angles[i]))});
			drawConnectedLines(newLine.concat(this.getArrowTip(newLine)), color);
		}
		// Fingers
		if(dof[3]){
			for(var f of this.gripper.fingers){
				const p = rotate2dVector([0,f.dirFactor*Math.abs(dof[3])*30],f.getAngle()); 
				var s = [addVec(f.getPos(true),p), f.getPos(true)]
				if(dof[3]<0){ s = s.reverse();}
				drawConnectedLines(s.concat(this.getArrowTip(s, 0.5)), color);
			}
		}
	}
	draw_dofs(){
		const colors = ['rgb(0,127,255)','rgb(172,229,238)'];
		for(var i=this.gripper.dofs.length-1; i>=0; i--){
			var dof = this.gripper.dofs[i];
			var index = this.gripper.chosenDofs.indexOf(i);
			if(index != -1){ // if dof is currently selected
				var color = colors[index];
				if(!this.hideDofOverlay) drawRect(33+i*50,58,45,80, color,0,false);
				this.draw_dof(dof, color);
			}
			if(this.hideDofOverlay) continue;
			for(var j=0; j<dof.length; j++){
				writeText(55+i*50,20+j*20,dof[j].toFixed(2),{align:'right'});
			}
			writeText(55+i*50,105, this.gripper.eigVals[i].toFixed(2),{align:'right'});
		}
	}
	draw_overlay(){
		// Dofs
		if(USE_MODEL && this.hudButton.value=="true"){
			this.draw_dofs();
		}
		if(this.hideDofOverlay)	return;
		// FPS
		this.frameCounter +=1;
		const t = Date.now();
		const dt = t - this.time;
		if(dt > 1000){ // check every second
			this.fps = this.frameCounter/dt*1000;
			this.time = t;
			this.frameCounter = 0;
		}
		writeText(canvas.width, canvas.height, `${this.fps.toFixed(2)} FPS`, {align:'right', baseline:'bottom'})
	}
	draw(){
		clear();
		this.simElements.forEach(e=>e.draw());
		this.draw_overlay();
	}
	handleJoystickInput(){
		if(this.joystick){
			var js = this.joystick()
			if(js.buttons[3].pressed && !this.joystick_oldButtons[3]) this.restart(); // 1 / Y / Triangle
			if(js.buttons[9].pressed && !this.joystick_oldButtons[9]) handle_recording(); // Start Button
			const axes = js.axes;
			const triggerV = js.buttons[7].value-js.buttons[6].value;
			const joystickInput = [-axes[1], axes[0], axes[2], triggerV];
			this.gripper.joystickInput = joystickInput.map((v)=>{return Math.abs(v)<0.01?0:(Math.abs(v)>0.99?1.0*Math.sign(v):v)});
			this.joystick_oldButtons = Array.from(js.buttons, e=>e.pressed);
		} else this.gripper.joystickInput = undefined;
	}
	update(dt){
		this.handleJoystickInput()
		this.simElements.forEach(e=>e.update(dt));
		this.world.Step(dt, 10, 8);
		this.world.ClearForces();
		this.simElements.forEach(e=>{if(e instanceof SimObject)e.returnToView()});
	}
	start(){
		const FPS = 30;
		const FPS_inv = 1.0/(FPS);
		
		this.running = true;
		const sim = this;
		var t0 = Date.now(), t1=0, dt=0;
		var gameloop = setInterval(function(){
			if(sim.sleeping) return;
			if (!sim.running){
				clearInterval(gameloop);
				writeText(canvas.width,0,"SIMULATION STOPPED!",{baseline:'top', align:'right', color:'red'});
				return 0;
			}
			t1 = Date.now();
			dt = (t1 - t0)/1000;
			t0 = t1;
			sim.update(dt)
			sim.draw()
		}, FPS_inv*1000); // call function with FPS time (*1000 to convert to ms)
	}
	cancel(){
		this.running = false;
	}
}

var sim = null
function main(){
	setupWebsite();
	sim = new Simulation2D();
	sim.setup();
	sim.start();
}

main();