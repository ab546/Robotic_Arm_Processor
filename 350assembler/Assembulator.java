import java.util.*;
import java.util.Scanner;
import java.io.*;

/**
* Simple parser that converts MIPS code into machine code using the ISA provided in instruction_codes.txt
* @author pf51
**/
public class Assembulator {

	private Map<String, String[]> instructions;

	public Assembulator() {

		instructions = new HashMap<String, String[]>();
		setupInstructionsMap();

	}

	  /*************************
	 *
	 * 	     	PARSING
	 *
	 **************************/

	// convert assembly into machine code
	private void mipsParser() {

		Scanner sc = null;
		try { sc = new Scanner(new File("mips.txt"));
		} catch (Exception e) { System.out.println("File not found"); }
		
		int counter = 0;
		printHeader();

		while (sc.hasNextLine()) {

			String line = sc.nextLine();
			if (!line.contains("#") && !line.isEmpty()) {

				System.out.print(counter + " : ");
				String[] split_line = line.split("[, ]+");
				String instruction = split_line[0];

				// get type
				String[] data = instructions.get(instruction);
				String type = data[0];
				printOpcode(instruction);
		
				// print rest of instruction
				printInstruction(split_line, type);
				System.out.println(";");
				counter++;

			}
			
		}

		printFooter();

	}

	  /*************************
	 *
	 * 	     	SETUP
	 *
	 **************************/

	// setup instruction file
	private void setupInstructionsMap() {

		Scanner sc = null;

		try { sc = new Scanner(new File("instruction_codes.txt"));
		} catch (Exception e) { System.out.println("File not found");}
		
		while (sc.hasNextLine()) {

			String line = sc.nextLine();
			String[] split_line = line.split("[ , ]+");
			String instruction = split_line[0];
			String[] info = split_line[1].split(":");
			instructions.put(instruction, info);

		}

	}

	

	/*************************
	 *
	 * 	  PRINTING METHODS
	 *
	 **************************/

	// print instruction
	private void printInstruction(String[] input, String type) {

		if (type.equals("R")) {

			printReg(input[1]); // rd
			printReg(input[2]); // rs
			printReg(input[3]); // rt
			printReg(input[3]); // shamt
			printALUOpcode(input); // ALU_op
			printZeros(2); // zeros

		} else if (type.equals("I")) {

			if (input.length == 4) {

				printReg(input[1]); // rd
				printReg(input[2]); // rs
				printImmediate(input[3]); // N

			} else {

				printReg(input[1]); // rd
				String[] x = input[2].split("\\(");
				printReg(x[1].substring(0, x[1].length()-1)); // rs
				printImmediate(x[0]); // N


			}

		} else if (type.equals("JI")) {

			printBinary(input[1], 27); // T

		} else if (type.equals("JII")) {

			printReg(input[1]); // rd
			printZeros(22);

		} else if (type.equals("NOP")) {

			printZeros(27);

		}

	}

	private void printImmediate(String i) {

		printBinary(i, 17);

	}

	// print ALU opcode
	private void printALUOpcode(String[] input) {

		String instruction = input[0];
		String[] data = instructions.get(instruction);
		String ALU_opcode = data[2];
		System.out.print(ALU_opcode);

	}

	// print zeros
	private void printZeros(int x) {
		for (int i = 0; i < x; i++) {
			System.out.print("0");
		}
	}

	// prints register value or N
	private void printReg(String v) {

		if (v.equals("$rstatus")) {v = "$r30";}
		if (v.equals("$ra")) {v = "$r31";}
		if (v.contains("$r")) { v = v.substring(2);}

		printBinary(v, 5);

	}

	// prints binary representation of b, with d digits length
	private void printBinary(String b, int d) {

		int value = Integer.valueOf(b);
		for (int i = d-1; i >= 0; i--) {
			int masked = value >> i;
			System.out.print(masked & 1);
		}

	}

	// given instruction, print opcode
	private void printOpcode(String i) {
		String[] info = instructions.get(i);
		System.out.print(info[1]);
	}

	// print header
	private void printHeader() {

		System.out.println();
		String[] header = {"DEPTH = 4096;", "WIDTH = 32;", "ADDRESS_RADIX = DEC;",
		"DATA_RADIX = BIN;", "CONTENT", "BEGIN"};
		for (String s : header) { System.out.println(s); }
		System.out.println();

	}

	// print footer
	private void printFooter() {
		System.out.println("\nEND\n");
	}


	// run
	public static void main(String[] args) {

		Assembulator a = new Assembulator();
		a.mipsParser();

	}


}