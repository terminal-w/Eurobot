void DriveTo(int E1tar, int E2tar) {
	bool happy = 0;
	while (!happy) {
		int E1cur = instruct(getE1);
		int E2cur = instruct(getE2);
		int S1 = 1 / 120 * abs(E1tar - E1cur) / E1tar;
		int S2 = 1 / 120 * abs(E1tar - E1cur) / E1tar;
		if (E1cur == E1tar) {
			happy = 1;
			S1 = 0;
		}
		else if (E1tar - E1cur < 0) {
			S1 -= 7;
		}
		else {
			S1 += 7;
		}
		if (E2cur == E2tar) {
			happy = 1;
			S2 = 0;
		}
		else if (E2tar - E2cur < 0) {
			S2 -= 7;
	}
		else {
			S2 += 7;
		}
		instruct(setS1, S1);
		instruct(setS2, S2);
#if debug == 1
		DEBUG.println("Speed Adjustment: S1, S2");
		DEBUG.print(S1, DEC);
		DBEUG.println(S2, DEC);
#endif
	}
#if debug ==1
	DEBUG.println("Because I'm Happy");
#endif
	return;
}
void target(int distance, int radius) {
#if debug == 1
	DEBUG.println("Targeting...");
#endif
	bool ccw = 0;
	int E1Tar;
	int E2Tar;
	if (radius == 0) {
		E1Tar = enc_target(distance);
		E2Tar = E1Tar;
#if debug == 1
		DEBUG.print("Straight Line, length: ");
		DEBUG.println(distance, DEC);
#endif
	}
	else {
		int Do = sweep(distance, abs(radius));
		int Di = sweep(distance, abs(radius), 1);
		int EoTar = enc_target(Do);
		int EiTar = enc_target(Di);
#if debug == 1
		DEBUG.print("Swept Radius, length, radius: ");
		DEBUG.print(distance, DEC);
		DEBUG.print(", ");
		DEBUG.print(radius, DEC);
#endif
		if (radius < 0) {
			E1Tar = EoTar;
			E2Tar = EiTar;
#if debug == 1
			DEBUG.println(" Anti-Clockwise");
#endif
		}
		else {
			E1Tar = EiTar;
			E2Tar = EoTar;
#if debug == 1
			DEBUG.println(" Clockwise");
#endif
		}
	}
	DriveTo(E1Tar, E2Tar);
	adjustment
	notify();
	return;
}