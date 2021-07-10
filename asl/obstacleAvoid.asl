safety(bumper).
@Bumper [atomic]
+bumper(pressed)
	<-	.broadcast(tell, avoid(bumper(pressed),stopped));
		drivexy(0,0).
