//Simple function to run in the Sphero Edu program that rolls the Sphero
//back and forth for experiment purposes

async function startProgram() {
	let heading = 0;
	while(true) {
		await roll(heading, 40, 5);
		await delay(2);
		heading = (heading + 180) % 360;
		await setHeading(heading);
	}
}
