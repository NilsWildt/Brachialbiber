package nilswildt.de;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class WriteFile {
	private File file;
	private FileWriter writer;

	public WriteFile() {
		file = new File("AAAoutput.txt");
		if (!file.exists()) {
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	public <T> void writeToFile(T data) {
		String stringData = new String(data.toString());
		try {
			writer = new FileWriter(file, true);
			BufferedWriter out = new BufferedWriter(writer);
			out.write(stringData);
			out.write(";");
			out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
