/**
 * This file provides functions for drawing on the canvas.
 */
const canvas = document.getElementById("sim_canvas");
const ctx = canvas.getContext("2d")
ctx.font = "15px Lucida Console" // Setup

function drawLine(x1,y1,x2,y2, color, lineWidth=1){
	ctx.save();
	ctx.beginPath();
	ctx.moveTo(x1,y1);
	ctx.lineTo(x2,y2);
	ctx.strokeStyle=color;
	ctx.lineWidth = lineWidth
	ctx.stroke();
	ctx.restore();
}

function drawConnectedLines(lines, color){
	ctx.save();
	ctx.beginPath();
	ctx.moveTo(lines[0][0], lines[0][1]);
	for(var l of lines.slice(1)){
		ctx.lineTo(l[0],l[1]);
	}
	ctx.strokeStyle=color;
	ctx.lineWidth = 4;
	ctx.stroke();
	ctx.restore();
}

function drawRect(x,y,w,h,color, angle, drawBoundary=true) {
	ctx.save();
	ctx.beginPath();
	ctx.translate(x, y);
	ctx.rotate(angle) // rotate needs angle in radians !
	ctx.fillStyle = color;
	ctx.fillRect(-w/2,-h/2,w,h);			
	if(drawBoundary){
		ctx.lineWidth=1;
		ctx.rect(-w/2,-h/2,w,h);
		ctx.stroke();
	}
	ctx.restore();
}
function drawCirc(x,y,radius, color){
	ctx.beginPath();
	ctx.arc(x,y, radius, 0, 2*Math.PI, false);
	ctx.fillStyle = color;
	ctx.fill();
	ctx.stroke();
}
function writeText(x,y,txt, {color='black', align='left', baseline='top'}={}){
	ctx.beginPath();
	ctx.fillStyle = color;
	ctx.textAlign = align
	ctx.textBaseline = baseline;
	ctx.fillText(txt,x,y);
}

function clear(){
	ctx.clearRect(0,0,canvas.width, canvas.height);
}