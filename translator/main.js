const example = decodeURI('%20%20LD%20X0%0A%20%20ANDN%20Y0%0A%20%20ST%20M1%0A%20%20LD%20X0%0A%20%20AND%20Y0%0A%20%20ST%20M2%0A%20%20LD%20M1%0A%20%20OR%20Y0%0A%20%20ANDN%20M2%0A%20%20ST%20Y0%0A%20%20END%0A');

const operations = ['NOP', 'LD_X', 'LD_Y', 'LD_M', 'LDN_X', 'LDN_Y', 'LDN_M', 'ST_Y', 'ST_M', 'STN_Y', 'STN_M', 'AND_X', 'AND_Y', 'AND_M', 'ANDN_X', 'ANDN_Y', 'ANDN_M', 'OR_X', 'OR_Y', 'OR_M', 'ORN_X', 'ORN_Y', 'ORN_M', 'XOR_X', 'XOR_Y', 'XOR_M', 'XORN_X', 'XORN_Y', 'XORN_M', 'NOT', 'ADD_K', 'ADD_D', 'SUB_K', 'SUB_D', 'MUL_K', 'MUL_D', 'DIV_K', 'DIV_D', 'MOD_K', 'MOD_D', 'PWM_K', 'PWM_D', 'MOV_K', 'MOV_D', 'END'];

function decToHex(d){
	let hex = d.toString(16);
	if (hex.length < 2) {
		hex = '0' + hex;
	}
	return "0x" + hex;
}

function lowByte(input){
	return input & 0xff;
}

function highByte(input){
	return (input>>8) & 0xff;
}

function parse(str){
	let cmdCode, parsed;
	let result = [];
	let cmd = str.toUpperCase().trim();
	
	//let instruction = /^(\w{2,4})(\s+([XYDMKTC]\d{1,5})(\s+([XYDMKTC]\d{1,5}))?)?$/;
	let instruction = /^(\w{2,4})(\s+(\w+)(\s+(\w+))?)?$/;
	let comment = /^;\s*(\w*)?$/;
	let label = /^([A-Z]+\w*):$/;
	
	if (parsed = cmd.match(instruction)) {
	
		let opCode = parsed[1];
		let operand1 = parsed[3];
		let operand2 = parsed[5];
		//console.log("opCode: " + opCode + "; op1: " + operand1 + "; op2: " + operand2);
		
		if (opCode == 'NOP') {
			if (typeof (operand1) === "undefined") {
				cmdCode = operations.indexOf('NOP');
				result.push(decToHex(cmdCode));
				console.info(result + '  ; ' + opCode);
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'LD') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('LD_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('LD_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('LD_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'LDN') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('LDN_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('LDN_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('LDN_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'ST') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([YM])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'Y') { cmdCode = operations.indexOf('ST_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('ST_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'STN') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([YM])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'Y') { cmdCode = operations.indexOf('STN_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('STN_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'AND') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('AND_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('AND_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('AND_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'ANDN') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('ANDN_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('ANDN_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('ANDN_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'OR') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('OR_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('OR_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('OR_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'ORN') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('ORN_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('ORN_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('ORN_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'XOR') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('XOR_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('XOR_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('XOR_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'XORN') {
			if (typeof (operand2) === "undefined") {
				let op;
				if (op = operand1.match(/^([XYMT])(\d{1,3})$/)) {
					let letter = op[1];
					let num = parseInt(op[2]);
					
					if (letter == 'X') { cmdCode = operations.indexOf('XORN_X'); }
					else if (letter == 'Y') { cmdCode = operations.indexOf('XORN_Y'); }
					else if (letter == 'M') { cmdCode = operations.indexOf('XORN_M'); }
					
					result.push(decToHex(cmdCode), decToHex(num));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand: ' + operand1);
				}
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'NOT') {
			if (typeof (operand1) === "undefined") {
				cmdCode = operations.indexOf('NOT');
				result.push(decToHex(cmdCode));
				console.info(result + '  ; ' + opCode);
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else if (opCode == 'ADD') {
			if (!(typeof (operand1) === "undefined" && typeof (operand2) === "undefined") ) {
				let op1, op2;
				if ((op1 = operand1.match(/^(D)(\d{1,3})$/)) && (op2 = operand2.match(/^([DK])(\d{1,5})$/))) {
					let letter2 = op2[1];
					let num1 = parseInt(op1[2]);
					let num2 = parseInt(op2[2]);
					
					if (letter2 == 'D') { cmdCode = operations.indexOf('ADD_D'); }
					else if (letter2 == 'K') { cmdCode = operations.indexOf('ADD_K'); }
					
					result.push(decToHex(cmdCode), decToHex(num1), decToHex(num2));
					
					console.info('[' + result + ']  ; ' + opCode + ' ' + operand1 + ' ' + operand2);
				}
				else {
					result.push(-2);
					console.error('Syntax error: unknown operand in ' + cmd);
				}
			}
			else {
				result.push(-4);
				console.error("Syntax error: expected operand in " + cmd);
			}
		}
		
		else if (opCode == 'END') {
			if (typeof (operand1) === "undefined") {
				cmdCode = operations.indexOf('END');
				result.push(decToHex(cmdCode));
				console.info(result + '  ; ' + opCode);
			}
			else {
				result.push(-3);
				console.error("Syntax error: extra operand in " + cmd);
			}
		}
		
		else {
			result.push(-1);
			console.error("Unknown command: " + cmd);
		}
		
		return result;
	}
	else if (parsed = cmd.match(comment)){
		console.info("Comment: " + parsed[1]);
	}
	else if (parsed = cmd.match(label)){
		console.info("Label: " + parsed[1]);
	}
	else {
		console.error("Syntax error: " + cmd);
	}
}
		
function toIntelHEX(startAddress, byteArray){
	
	
}
		
window.onload = function() {
	let bytecode = document.getElementById("bytecode");

	var ed = CodeMirror.fromTextArea(translator.editor, {
        theme: "cobalt",
		tabSize: 2,
		lineNumbers: true
      });
	  
	ed.setValue(example);
	
	translator.translate.onclick = function(){
		var lines = ed.getValue().replace(/\r\n/g,"\n").split("\n").filter(line => line);
		
		//console.log(lines);
		
		bytecode.value = "";
		
		let statusTable = document.getElementById("status").children[1];
		
		statusTable = clearTable(statusTable);
		
		let bytecodeArray = [];
		
		for(let i=0; i<lines.length; i++){
			let item = parse(lines[i]);
			if (item == -1){
				let msg = 'Unknown command: ' + lines[i];
				addRow(statusTable, [" ", (i+1), msg]);
			}
			else if (item == -2){
				let msg = 'Syntax error: unknown operand in ' + lines[i];
				addRow(statusTable, [" ", (i+1), msg]);
			}
			else if (item == -3){
				let msg = 'Syntax error: extra operand in ' + lines[i];
				addRow(statusTable, [" ", (i+1), msg]);
			}
			else if (item == -4){
				let msg = 'Syntax error: expected operand in ' + lines[i];
				addRow(statusTable, [" ", (i+1), msg]);
			}
			else {
				item.forEach(function(byte){
					bytecodeArray.push(byte);
				});
				//bytecode.value += item;
			}
		}
		
		bytecode.value = bytecodeArray.join(', ');
		
		
				
		//lines.forEach(parse);

	}
	
	
	

	function addRow(id, cells){
		// Insert a row in the table at row index 0
		var newRow = id.insertRow(-1);
		
		cells.forEach(function(cell){
			// Insert a cell in the row at index 0
			let newCell = newRow.insertCell(-1);
			// Append a text node to the cell
			let newText = document.createTextNode(cell);
			newCell.appendChild(newText);
		});
	}
	
	function clearTable(old_tbody){
		var new_tbody = document.createElement('tbody');
		old_tbody.parentNode.replaceChild(new_tbody, old_tbody);
		return new_tbody;
	}
	
}