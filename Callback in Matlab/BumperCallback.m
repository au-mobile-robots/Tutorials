function BumperCallback(src,msg)
  global BumperMsg
  global BumperEvent
  
  %disp(msg.State)
  BumperMsg = msg;
  BumperEvent = true;
end

