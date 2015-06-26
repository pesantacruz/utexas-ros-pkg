package com.example.pesantacruz.networkalignment;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;

/**
 * Created by Michael on 6/16/2015.
 */
public class LoggingManager {
    private Map<String,Stat> sendlist;
    private Map<String, Stat> receivelist;
    private String log = "";
    private DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");

    public LoggingManager(){
        sendlist = new HashMap<String, Stat>();
        receivelist = new HashMap<String, Stat>();
    }

    public void sendTo(String address, int size){
        if(sendlist.containsKey(address)){
            sendlist.put(address, sendlist.get(address).increaseCount().increaseBytes(size));
        }
        else{
            sendlist.put(address, new Stat().increaseCount().increaseBytes(size));
        }
        log += dateFormat.format(new Date()) + "\tSent a " + size + "Bytes message to: " + address + "\n";

    }
    public void receiveFrom(String address, int size){
        if(receivelist.containsKey(address)){
            receivelist.put(address, receivelist.get(address).increaseCount().increaseBytes(size));
        }
        else{
            receivelist.put(address, new Stat().increaseCount().increaseBytes(size));
        }
        log += dateFormat.format(new Date()) + "\tReceived a " + size + "Bytes message from: " + address + "\n";
    }
    public String getLog(){
        String result = "";
        result += "Sent\n";
        for(String key : sendlist.keySet()){
            result += key + "\t\t" + sendlist.get(key).getCount()+"messages\t\t " + sendlist.get(key).getNumbytes() +"Bytes" + "\n";
        }
        result += "\nReceived\n";
        for(String key : receivelist.keySet()){
            result += key + "\t\t" + receivelist.get(key).getCount()+"messages\t\t " + sendlist.get(key).getNumbytes() +"Bytes" + "\n";
        }
        return result + log;
    }

    public class Stat{
        private int numbytes;
        private int count;

        public Stat(){
            numbytes = 0;
            count = 0;
        }

        public Stat increaseBytes(){
            numbytes += 1;
            return this;
        }
        public Stat increaseCount(){
            count += 1;
            return this;
        }
        public Stat increaseBytes(int num){
            numbytes += num;
            return this;
        }
        public Stat increaseCount(int num){
            count += num;
            return this;
        }
        public int getNumbytes(){
            return numbytes;
        }
        public int getCount(){
            return count;
        }
    }

}
