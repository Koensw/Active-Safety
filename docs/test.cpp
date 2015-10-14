listener(){    
    subscriber.listen("subscriber channel")
    subscriber.listen("service channel")
    
    while(true){
        message = subscriber.nextMessage()
        
        if(message.channel == "service"){
            switch(message.type){
                case 'position':
                    x = message.readInt()
                    y = message.readInt()
                    z = message.readInt()
                    
                    setPosition(x,y,z)
                case 'target'
                    x = message.readInt()
                    y = message.readInt()
                    z = message.readInt()
                    
                    setTarget(x,y,z)
            }
        }else{
            switch(message.type){
                case 'position':
                    x, y, z = getPosition()
                    
                    return_message.addInt(x)
                    return_message.addInt(y)
                    return_message.addInt(z)
                    
                    subscriber.reply(return_message)
            } 
        }
    }
}

setTarget(){
    publisher.setAddress("subscriber channel")
    message.type = "position";
    message.addInt(x)
    message.addInt(y)
    message.addInt(z)
}

getPosition(){
    service.setAddress("service channel")
    return_message = service.send(message);
    read message
}

int updateTarget(){
    while(true){
        setTarget();
        wait(1);
    }
}