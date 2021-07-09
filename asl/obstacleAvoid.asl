safety(bumper).
@Bumper [atomic]
+bumper(pressed)
	<-	drive(bleft);
		turn(left);
		drive(forward).
