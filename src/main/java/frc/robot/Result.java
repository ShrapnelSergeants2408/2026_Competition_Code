package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Immutable result object returned by subsystem self-tests.
 *
 * <p>Each test method returns one {@code Result} describing whether a single
 * unit under test passed or failed, along with optional detail strings.
 * {@link #toString()} renders the result with ANSI color (green = pass,
 * red = fail) for easy console scanning.
 *
 * <p>Use the static factory methods rather than the constructor:
 * <pre>
 *   return Result.pass("Left encoder");
 *   return Result.fail("Right encoder", "velocity reads zero at full speed");
 * </pre>
 */
public class Result {
    /** Name of the unit or check that was tested. */
    public String unit;
    /** {@code true} if the test passed; {@code false} if it failed. */
    public boolean success;
    /** Optional detail messages providing context about the result. */
    public List<String> reports;

    private Result(String unit, boolean success, List<String> reports){
        this.unit = unit;
        this.success = success;
        this.reports = reports;
    }

    /**
     * Creates a passing result with no detail messages.
     *
     * @param unit name of the unit under test
     * @return a passing {@code Result}
     */
    public static Result pass(String unit){
        return new Result(unit,true, new ArrayList<>());
    }

    /**
     * Creates a passing result with one or more detail messages.
     *
     * @param unit    name of the unit under test
     * @param reports optional detail strings
     * @return a passing {@code Result}
     */
    public static Result pass(String unit, String... reports){
        return new Result(unit,true, Arrays.asList(reports));
    }

    /**
     * Creates a failing result with no detail messages.
     *
     * @param unit name of the unit under test
     * @return a failing {@code Result}
     */
    public static Result fail(String unit){
        return new Result(unit,false, new ArrayList<>());
    }

    /**
     * Creates a failing result with one or more detail messages.
     *
     * @param unit    name of the unit under test
     * @param reports detail strings describing why the test failed
     * @return a failing {@code Result}
     */
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
    
    /** @return {@code true} if the test passed */
    public boolean isSuccess(){
        return this.success;
    }

    /** @return {@code true} if the test failed */
    public boolean isFailure(){
        return !this.success;
    }
}
