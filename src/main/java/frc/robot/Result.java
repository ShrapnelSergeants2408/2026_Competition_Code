package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

// test function return info
public class Result {
    public String unit;
    public boolean success;
    public List<String> reports;
    private Result(String unit, boolean success, List<String> reports){
        this.unit = unit;
        this.success = success;
        this.reports = reports;
    }

    public static Result pass(String unit){
        return new Result(unit,true, new ArrayList<>());
    }
    public static Result pass(String unit, String... reports){
        return new Result(unit,true, Arrays.asList(reports));
    }
    public static Result fail(String unit){
        return new Result(unit,false, new ArrayList<>());
    }
    public static Result fail(String unit, String... reports){
        return new Result(unit,false, Arrays.asList(reports));
    }

    @Override
    public String toString(){
        final String RESET = "\\u001B[0m";
        final String GREEN = "\\u001B[32m";
        final String RED = "\\u001B[31m";
        String color;
        String status;
        
        if (success){ 
            color=GREEN;
            status="Success!";
        } else {
            color=RED;
            status="Failure!";
        }

        return String.format(">> %s %s %s: %s\n", 
            color,status,RESET,this.unit
        ) + this.reports.stream().map(s -> String.format(">>\t%s\n\n",s)).collect(Collectors.joining());
    }
    
    public boolean isSuccess(){
        return this.success;
    }
    public boolean isFailure(){
        return !this.success;
    }
}
