// Component behaviours (think in terms of state)

{ include("map.asl") }


movement(waypoint).
+!waypoint(Location)
    :   atLocation(Location,_)
    <-  .print(atLocation(Location)).
		//.broadcast(tell, waypoint(atLocation(Location,Range)));
		//.print(drive(stop)).

+!waypoint(Location)
	:	locationName(Location,[X,Y])
    <-  !faceNext(Location);
		!drive(forward);
		-position(_,_);
		+position(X,Y);
		!waypoint(Location).

+!faceNext(Next)
	:	direction(CurrentDirection)
		& directionToNext(Next,NewDirection)
		& face(CurrentDirection,NewDirection,TurnAction)
	<-	!turn(TurnAction);
		-direction(_);
		+direction(NewDirection).
		
+!faceNext(_).
		
directionToNext(Next,e)
	:-	position(X1,_)
		& locationName(Next,[X2,_])
		& X1 < X2.

directionToNext(Next,w)
	:-	position(X1,_)
		& locationName(Next,[X2,_])
		& X1 > X2.	
		
directionToNext(Next,n)
	:-	position(_,Y1)
		& locationName(Next,[_,Y2])
		& Y1 < Y2.

directionToNext(Next,s)
	:-	position(_,Y1)
		& locationName(Next,[_,Y2])
		& Y1 > Y2.	
		

face(OldDirection,NewDirection,around)
	:-	((OldDirection == s) & (NewDirection == n))
		| ((OldDirection == n) & (NewDirection == s))
		| ((OldDirection == e) & (NewDirection == w))
		| ((OldDirection == w) & (NewDirection == e)).
	
face(OldDirection,NewDirection,left)
	:-	((OldDirection == s) & (NewDirection == e))
		| ((OldDirection == e) & (NewDirection == n))
		| ((OldDirection == n) & (NewDirection == w))
		| ((OldDirection == w) & (NewDirection == s)).
		
face(OldDirection,NewDirection,right)
	:-	((OldDirection == s) & (NewDirection == w))
		| ((OldDirection == w) & (NewDirection == n))
		| ((OldDirection == n) & (NewDirection == e))
		| ((OldDirection == e) & (NewDirection == s)).
		
face(OldDirection,NewDirection,ok)
	:-	OldDirection == NewDirection.	
	
+!turn(left)
	:	turnRate(X,Y)
	<-	!move(X,Y,2).

+!turn(right)
	: 	turnRate(X,Y)
	<-	!move(0-X,0-Y,2).

+!turn(around)
	:	turnRate(X,Y)
	<-	!move(X,Y,4).

+!turn(_)
	<-	.print(turn(default)).

+!drive(forward)
	:	driveRate(X,Y)
	<-	!move(X,Y,5).
	
+!drive(backward)
	:	driveRate(X,Y)
	<-	!move(0-X,0-Y,5).
	
+!drive(_)
	<-	.print(drive(default)).
	
	
+!move(X,Y,Count)
	: 	Count > 0
	<-	.print(drive(X,Y));
		!move(X,Y,Count-1).
	
+!move(_,_,_)
	<-	.print(move(default)).

	
/**
 * Turning and driving rates
 */
turnRate(0,0.41).
driveRate(0.5,0).



