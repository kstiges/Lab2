package edu.cmu.ri.mrpl.util;

public class RingBuffer<T> {
	int capacity;
	T[] array;
	int index;
	
	@SuppressWarnings("unchecked")
	public RingBuffer (int capacity) {
		this.capacity = capacity;
		array = (T[]) new Object[capacity];
		index = 0;
	}
	
	public void add (T obj) {
		array[index++ % capacity] = obj;
	}
	
	public T get (int i) {
		return array[(i + capacity) % capacity];
	}
	
	public int getCapacity () {
		return capacity;
	}
}