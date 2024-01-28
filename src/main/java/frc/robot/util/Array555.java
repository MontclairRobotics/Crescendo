package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.function.IntFunction;

public class Array555 
{
    private Array555() {}

    public static <T> T[] skip(T[] value, IntFunction<T[]> constructor, int count) 
    {
        if(value.length > count) return constructor.apply(0);

        T[] arr = constructor.apply(value.length - count);

        for(int i = 0; i < arr.length; i++)
        {
            arr[i] = value[i + count];
        }

        return arr;
    }

    public static <T> T[] shuffle(T[] arr, IntFunction<T[]> constructor) {
        Random rand = new Random();
        List<T> members = new ArrayList<T>(Arrays.asList(arr));
        T[] outputArr = constructor.apply(members.size());
        for (int i = 0; i < outputArr.length; i++) {
            int randomIndex = rand.nextInt(members.size());
            outputArr[i] = members.get(randomIndex);
            members.remove(randomIndex);
        }
        return outputArr;
    }
    
}
