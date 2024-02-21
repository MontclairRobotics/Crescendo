package frc.robot.util;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Consumer;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;

public abstract class Tunable<T> implements Sendable
{
    T value;

    ArrayList<Consumer<T>> updators = new ArrayList<>();

    public final GenericEntry entry;
    
    final String name;

    protected abstract void setNT(T value);
    protected abstract T getFromNT();

    protected Tunable(T value, String name)
    {
        this.value = value;
        this.name = name;

        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Tuning");
        entry = nt.getTopic(name).getGenericEntry();

        setNT(value);
        
        nt.addListener(name, EnumSet.of(Kind.kValueAll), (table, key, event) -> 
        {
            DriverStation.MatchType matchType = DriverStation.getMatchType();

            if(matchType == DriverStation.MatchType.None 
            || matchType == DriverStation.MatchType.Practice 
            || DriverStation.isTest())
            {
                this.value = getFromNT();

                for(Consumer<T> updator : updators)
                {
                    updator.accept(this.value);
                }
            }
        }); 
    }

    public static Tunable<Double> of(double value, String name) 
    {
        return new Tunable<Double>(value, name) 
        {
            @Override
            protected Double getFromNT() 
            {
                return entry.getDouble(value);
            }

            @Override
            protected void setNT(Double value) 
            {
                entry.setDouble(value);
            }

            @Override
            public void initSendable(SendableBuilder builder) 
            {
                builder.addDoubleProperty("value", this::get, this::setNT);
            }
        };
    }
    public static Tunable<Boolean> of(boolean value, String name)
    {
        return new Tunable<Boolean>(value, name) 
        {
            @Override
            protected Boolean getFromNT() 
            {
                return entry.getBoolean(value);
            }

            @Override
            protected void setNT(Boolean value) 
            {
                entry.setBoolean(value);
            }

            @Override
            public void initSendable(SendableBuilder builder) 
            {
                builder.addBooleanProperty("value", this::get, this::setNT);
            }
        };
    }

    public Tunable<T> whenUpdate(Consumer<T> updator)
    {
        updators.add(updator);
        return this;
    }

    public T get()
    {
        return value;
    }
}
