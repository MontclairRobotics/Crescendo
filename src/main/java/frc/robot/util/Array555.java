package frc.robot.util;

public class Array555 {
    public static int indexOf(char[] arr, char value) {
        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == value) {
                return i;
            }
        }
        return -1;
    }
}
