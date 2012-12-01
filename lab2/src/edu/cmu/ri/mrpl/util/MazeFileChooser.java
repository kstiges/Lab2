package edu.cmu.ri.mrpl.util;

import java.io.File;

import javax.swing.JFileChooser;

public class MazeFileChooser extends JFileChooser {
	public MazeFileChooser () {
		javax.swing.filechooser.FileFilter mazeFilter = new javax.swing.filechooser.FileFilter() {
			public boolean accept(File file) {
				if (file.isDirectory())
					return true;

				return file.getName().endsWith(".maze");
			}

			public String getDescription() {
				return "Maze files (*.maze)";
			}
		};
		addChoosableFileFilter(mazeFilter);
		setFileFilter(mazeFilter);
	}
}
