/**
 * Primary mission: collect and deliver mail
 * Plan is simple: 
 * (1) Make mental note that I have a mail mission
 * (2) Collect the mail
 * (3) Deliver the mail
 * (4) Delete mental note about mail mission
 * Updating the user at every step.
 */
 
 
mission(mission).
+!mission(Goal,Parameters)
    :   Goal = 'collectAndDeliverMail'
        & Parameters = [Sender,Receiver]
    <-  +mission(Goal,Parameters);
        !collectAndDeliverMail(Sender,Receiver)
        -mission(Goal,Parameters).
 
		
mission(collectAndDeliverMail).
+!collectAndDeliverMail(Sender,Receiver)
	<-	.broadcast(tell, mailUpdate(receivedMission,Sender,Receiver));
		!collectMail(Sender);
		.broadcast(tell, mailUpdate(gotMail,Sender,Receiver));
		!deliverMail(Receiver);
		.broadcast(tell, mailUpdate(delivered,Receiver)).

/**
 * !collectMail(Sender)
 * Go get the mail from the sender
 */
mission(collectMail).

// I do not yet have the mail, go get it.
+!collectMail(Sender)
	:	not haveMail
	<-	.broadcast(tell, collectMail(goToSender,Sender));
		!navigate(Sender);
		.broadcast(tell, collectMail(atSender,Sender));
		+haveMail;
		.broadcast(tell, collectMail(haveMail,Sender)).

// I already have the mail.
+!collectMail(Sender)
	:	haveMail
	<-	.broadcast(tell, collectMail(alreadyHaveMail,Sender)).

/**
 * !deliverMail(Receiver)
 * Give the mail to the receiver
 */
mission(deliverMail).
 
 // Case where I have the mail.
+!deliverMail(Receiver)
	:	haveMail
	<-	.broadcast(tell, deliverMail(goToReceiver,Receiver));
		!navigate(Receiver);
		.broadcast(tell, deliverMail(arrivedAtReceiver,Receiver));
		-haveMail;
		.broadcast(tell, deliverMail(delivered,Receiver)).
		
// Case where there is no mail to deliver.
+!deliverMail(Receiver)
	:	not haveMail
	<-	.broadcast(tell, deliverMail(noMail,Receiver)).
	
/**
 * Default plans.
 * These can run when unrelated perceptions are received, resulting in a 
 * reasoning cycle where no plan context is applicable for that reasoning cycle.
 */
 // Ensure recursion and remember the parameters.
 // Note: This plan should never run as there are no context guards in the 
 // main plan for this goal above. If this plan runs, something strange 
 // happened.
+!collectAndDeliverMail(Sender,Receiver)
	<-	.broadcast(tell, collectAndDeliverMail(default));
		!collectAndDeliverMail(Sender,Receiver).
		
		
// Default plan for collect mail. Should not be possible to get here.
+!collectMail(Sender)
	<-	.broadcast(tell, collectMail(error,Sender)).
		
// Default plan for deliver mail. Should not be possible to get here.
+!deliverMail(Receiver)
	<-	.broadcast(tell, deliverMail(error,Receiver)).

