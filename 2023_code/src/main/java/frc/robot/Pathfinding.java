package frc.robot;
import java.util.*;
public class Pathfinding
{
    int[][] arena;
    int state = 0;
    public void makeMap(int width, int height)
    {
        arena = new int[26][54];
        System.out.println(arena[][]);
    }

    public int[][] getMap()
    {
        return arena;
    }

    public int getSatus(int x , int y)
    {
        return arena[x][y];
    }



}