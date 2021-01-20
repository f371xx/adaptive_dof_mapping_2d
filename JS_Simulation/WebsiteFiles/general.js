/** 
 * This file holds all non-simulation related implementations
 * like button-actions.
 */

/* Websocket events */
const MODEL_NAME_REQUEST = "modelNames",
	DOF_REQUEST = "dofs",
	ABOUT_REQUEST = "about",
	GET_MODEL_VECTORS = "GetModelVectors";
const START_RECORDING = "StartRecording",
	STORE_STATE = "StoreState",
	STOP_RECORDING = "StopRecording",
	RESTARTING_SIM = "RestartingSimulation";
		
const ioSocket = io();
var USE_MODEL;	// defined by button, weather to use adaptive control or not
var modelsLoadedPromise = undefined;

function setupWebsite(){
	modelsLoadedPromise = new Promise( (resolve)=>{
		ioSocket.emit(MODEL_NAME_REQUEST,{}, (data)=>{
			var modelSelector = document.getElementById('modelSelect');
			for(var i=0; i<data.length;i++){
				modelSelector.options[i] = new Option(data[i],data[i]);
				if(data[i] == "default") modelSelector.selectedIndex = i;
			}
			resolve();
		});
	})
	toggleExplanationText(document.getElementById("explainButton"))
	toggleAdaptiveButton()
}	

function toggleExplanationText(){
	const explanation = document.getElementById('explanation');
	const explainButton = document.getElementById('explainButton');
	const texts = ["...", "hide explanation"];
	if(explainButton.innerHTML == texts[0]){
		explainButton.innerHTML = texts[1];
		explanation.style.display = "block";
	} else {
		explainButton.innerHTML = texts[0];
		explanation.style.display = "none";		
	}
}

function aboutButton_pressed(){
	ioSocket.emit(ABOUT_REQUEST, document.getElementById('modelSelect').value,
	 (data)=>{
		 alert(data);
	 });
}

function handle_recording(){
	const recordingButton = document.getElementById('recordingButton');
	if(recordingButton.value === "true"){
		recordingButton.value = false;
		recordingButton.textContent = "Start Recording";
		var comment = prompt("Please enter a short comment for this recording.");
		while(comment =="" || comment ==null) comment = prompt("Please enter a short comment for this recording. Do not press Cancel or leave field empty.");
		ioSocket.emit(STOP_RECORDING, comment+`${document.getElementById('poleButton').value=='true'?"; poles used":"; no poles"}`, (response)=>{
			alert(response)
		});
	} else {
		recordingButton.value = true;
		recordingButton.textContent = "Stop Recording";
		ioSocket.emit(START_RECORDING);
	}
}

function togglePoles(){
	if(document.getElementById('recordingButton').value!='true'){
		const poleButton = document.getElementById('poleButton');
		poleButton.value=!(poleButton.value=='true');
		sim.boxes.forEach(e=>e.togglePole(poleButton.value=='true'))
	} else alert("You cannot toggle poles while recording.")
}

function toggleHUD(){
	const hudButton = document.getElementById('toggleHudButton');
	hudButton.value = (hudButton.value != "true")
}

function toggleAdaptiveButton(){
	const adaptiveButton = document.getElementById("adaptiveButton");
	const adaptiveDiv = document.getElementById('adaptiveDiv')
	const texts = ["Enable Adaptive Control", "Disable Adaptive Control"];
	if(adaptiveButton.innerHTML == texts[0]){
		adaptiveButton.innerHTML = texts[1];
		adaptiveDiv.style.display = "block";
		USE_MODEL=true;
	} else {
		adaptiveButton.innerHTML = texts[0];
		adaptiveDiv.style.display = "none";
		USE_MODEL=false;
	}
}

function getModelVectors(){
	ioSocket.emit(GET_MODEL_VECTORS, {'model':document.getElementById('modelSelect').value,
		'state':sim.gripper.getState().slice(4)},(response)=>{
			console.log(response)});
}