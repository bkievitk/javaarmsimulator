package com.armsim.my3DShapes;


import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.Vector;

import javax.imageio.ImageIO;

import com.armsim.my3DCore.Material;
import com.armsim.my3DCore.TransformMy3D;

public class Ply3D extends Object3D {
	
	private static final long serialVersionUID = 92347860010L;

	public static final int FORMAT_ASCII = 1;
	public static final int FORMAT_BINARY_BIG_ENDIAN = 2;
	public static final int FORMAT_BINARY_LITTLE_ENDIAN = 3;

	public static final int ELEMENT_VERTEX = 0;
	public static final int ELEMENT_FACE = 1;
	public static final int ELEMENT_PASS = 2;
	
	public Ply3D(File file) {
		try {

			// Data.
			String headerLine;
			int format = FORMAT_ASCII;
			String formatVersion;
			Vector<PLYElement> elements = new Vector<PLYElement>();
			Vector<BufferedImage> textures = new Vector<BufferedImage>();
			
			boolean loadTextures = false;
			boolean loadFaces = true;
			boolean verbose = true;     
			
			// We need the parent directory too look up ancilarity image files.
			File workingDir = file.getParentFile();
						
			// Reader.
			//BufferedReader r = new BufferedReader(new FileReader(file));
			DataInputStream r = new DataInputStream(new FileInputStream(file));
			
			// Check magic number.
			if(!r.readLine().equals("ply")) {
				System.out.println("Ply file must start with the magic number 'ply'.");
				return;
			}
			
			// Read header section.
			while(!(headerLine = r.readLine()).equals("end_header")) {
				
				// Header is broken into meaningful lines.
				String[] headerInfo = headerLine.split(" ");
				
				// Check format.
				if(headerInfo[0].equals("format")) {
					
					// format ascii/binary version
					if(headerInfo.length != 3) {
						System.out.println("Format must be in the order 'format ascii/binary version'.");
						return;
					}
					
					if(headerInfo[1].equals("ascii")) {
						format = FORMAT_ASCII;
						if(verbose) {System.out.println("File type is ASCII");}
					} else if(headerInfo[1].equals("binary_little_endian")) {
						format = FORMAT_BINARY_LITTLE_ENDIAN;
						if(verbose) {System.out.println("File type is little endian");}						
					} else if(headerInfo[1].equals("binary_big_endian")) {
						format = FORMAT_BINARY_BIG_ENDIAN;
						if(verbose) {System.out.println("File type is big endian");}						
					} else {
						System.out.println("Format must be ascii or binary.");
						return;
					}
					formatVersion = headerInfo[2];

				// Information on object.
				// This is for image mapping.
				} else if(headerInfo[0].equals("obj_info")) {
					if(headerInfo.length != 2) {
						System.out.println("Image must be in the order 'obj_info imagename'.");
						return;
					}
					if(verbose) {System.out.println("Object info found.");}
					
					File imgLocation = new File(workingDir.getAbsolutePath() + "/" + headerInfo[1]);
					
					try {
						textures.add(ImageIO.read(imgLocation));
					} catch(Exception e) {
						System.out.println("Unable to read image [" + imgLocation + "]");
						return;
					}
					
				// This is an element (node or vertex)
				} else if(headerInfo[0].equals("element")) {
					if(headerInfo.length != 3) {
						System.out.println("Element information must contain 3 values.");
						return;
					}
					
					// Types of elements.
					if(headerInfo[1].equals("vertex")) {
						elements.add(new PLYElement(ELEMENT_VERTEX));
						if(verbose) {System.out.println("Vertex element found.");}
					} else if(headerInfo[1].equals("face")) {
						elements.add(new PLYElement(ELEMENT_FACE));
						if(verbose) {System.out.println("Face element found.");}
					} else if(headerInfo[1].equals("pass")) {
						elements.add(new PLYElement(ELEMENT_PASS));
						if(verbose) {System.out.println("Pass element found.");}
					} else {
						System.out.println(headerInfo[1] + " is an invalid element name.");
						return;
					}
					
					// Try to extract the counts of that element.
					try {
						elements.lastElement().count = Integer.parseInt(headerInfo[2]);
						if(verbose) {System.out.println(elements.lastElement().count + " values to be found for element.");}
					} catch(NumberFormatException e) {
						System.out.println("Element must contain a valid number.");
						return;
					}
					
				// Properties are found within an element.
				// They are the data types of that element.
				} else if(headerInfo[0].equals("property")) {
					if(elements.size() < 1) {
						System.out.println("An element must be specified before properties can be set.");
						return;
					}
					
					if(headerInfo.length < 3) {
						System.out.println("Property information must contain at least 3 values.");
						return;
					}
					
					// Should do more specific here.
					if(headerInfo[1].equals("list")) {
						PLYProperty property = new PLYProperty(headerInfo[4],headerInfo[3],headerInfo[2]);
						elements.lastElement().properties.add(property);
						if(verbose) {System.out.println("List property found.");}
					} else {
						PLYProperty property = new PLYProperty(headerInfo[2],headerInfo[1]);
						elements.lastElement().properties.add(property);
						if(verbose) {System.out.println("Single property found. " + elements.lastElement());}
					}
				} else if(headerInfo[0].equals("comment")) {
					// Just a comment.
				}
			}
			
			if(verbose) {System.out.println("Header finished.");}
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			// Finished reading the header.
			for(PLYElement element : elements) {
				
				if(verbose) {System.out.println("Reading element " + element);}
				
				int vertexCount = 0;
				int faceCount = 0;
				int passCount = 0;
				
				for(int i = 0; i < element.count; i++) {
					
					//if(verbose) {System.out.println("Reading element object " + i);}
					
					// Convert string into list of doubles.
					Object[] vals = getVals(r,format,element);
					
					if(vals == null) {
						System.out.println("Error reading points.");
						return;
					}
					
					//if(verbose) {System.out.println(vals.length + " values found.");}
					
					if(element.type == ELEMENT_VERTEX) {
						vertexCount++;
						
						double[] pts = new double[3];
						double[] ptsNrm = new double[3];
						
						int index = 0;
						for(PLYProperty property : element.properties) {
							
							//System.out.println(property);
							
							if(property.name.equals("x")) {
								pts[0] = (Double)vals[index];
							} else if(property.name.equals("y")) {
								pts[1] = (Double)vals[index];									
							} else if(property.name.equals("z")) {
								pts[2] = (Double)vals[index];									
							} else if(property.name.equals("nx")) {
								ptsNrm[0] = (Double)vals[index];									
							} else if(property.name.equals("xy")) {
								ptsNrm[1] = (Double)vals[index];									
							} else if(property.name.equals("nz")) {
								ptsNrm[2] = (Double)vals[index];									
							}
							index++;
						}
						
						points.add(pts);
						ptNormals.add(ptsNrm);
					} else if(element.type == ELEMENT_FACE) {
						faceCount++;
						
						if(loadFaces) {
							int index = 0;
							for(PLYProperty property : element.properties) {
								if(property.isList()) {
									Object[] list = (Object[])vals[index];
									
									if(property.name.equals("vertex_indices")) {
										if(list.length == 3) {
											
											int[] triangle = new int[3];
	
											triangle[0] = (Integer)list[0];
											triangle[1] = (Integer)list[1];
											triangle[2] = (Integer)list[2];
											
											triangles.add(triangle);
											triangleMaterial.add(new Material(Color.RED, "Ply", null));
											
										} else if(list.length == 4) {
											
											int[] triangle1 = new int[3];
											int[] triangle2 = new int[3];
											
											triangle1[0] = (Integer)list[0];
											triangle1[1] = (Integer)list[1];
											triangle1[2] = (Integer)list[2];
											
											triangle2[0] = (Integer)list[1];
											triangle2[1] = (Integer)list[2];
											triangle2[2] = (Integer)list[3];
											
											triangles.add(triangle1);
											triangleMaterial.add(new Material(Color.RED, "Ply", null));
											triangles.add(triangle2);
											triangleMaterial.add(new Material(Color.RED, "Ply", null));
											
										} else {
											System.out.println("Can not handle this many points yet [" + list.length + "].");
											return;
										}
									}
									index++;
								} else {
									// Don't know how to handle single elements in a face.
								}
							}
						}
					} else if(element.type == ELEMENT_PASS) {
						passCount++;
						
						int index = 0;
						int imgIndex = -1;
						int[] faceIndexs = null;
						
						for(PLYProperty property : element.properties) {
							
							if(property.isList()) {
								
								Object[] list = (Object[])vals[index];
																
								if(property.name.equals("face_indices")) {
									if(verbose) {System.out.println("Face indicies found");}
									faceIndexs = new int[list.length];
									
									
									for(int j=0;j<list.length;j++) {
										faceIndexs[j] = (Integer)list[j];
									}
								}
								
								index ++;
								
							} else {
								if(property.name.equals("tex_index")) {
									if(verbose) {System.out.println("Tex index found. [" + vals[index] + "]");}
									imgIndex = (Integer)vals[index];
								}
								index++;
							}
						}
						
						if(true) { //passCount == 1) {
							if(loadTextures && imgIndex >= 0 && imgIndex < textures.size() && faceIndexs != null) {
								// Need to have image index and face index.
								BufferedImage img = textures.get(imgIndex);
								if(faceIndexs.length == 4) {
	
									System.out.println("Building texture face.");
									
									//Material material1 = new Material(Color.BLUE, "Ply", img);
									//Material material2 = material1.newImage(false, true, true, true);
									
									Material material3 = new Material(Color.BLUE, "Ply", null);
									Material material4 = new Material(Color.BLUE, "Ply", null);
									Material material1 = new Material(Color.BLUE, "Ply", null);
									Material material2 = new Material(Color.BLUE, "Ply", null);
									
									int[] triangle1 = new int[3];
									int[] triangle2 = new int[3];

									/*
									System.out.println(faceIndexs[0] + " " + faceIndexs[1] + " " + faceIndexs[2] + " " + faceIndexs[3]);
									System.out.println(	"Pts " +
														points.get(faceIndexs[0])[0] + " " +
														points.get(faceIndexs[0])[1] + " " +
														points.get(faceIndexs[0])[2]);
									System.out.println(	"Pts " +
														points.get(faceIndexs[1])[0] + " " +
														points.get(faceIndexs[1])[1] + " " +
														points.get(faceIndexs[1])[2]);
									System.out.println(	"Pts " +
														points.get(faceIndexs[2])[0] + " " +
														points.get(faceIndexs[2])[1] + " " +
														points.get(faceIndexs[2])[2]);
									System.out.println(	"Pts " +
														points.get(faceIndexs[3])[0] + " " +
														points.get(faceIndexs[3])[1] + " " +
														points.get(faceIndexs[3])[2]);
									*/
									
									triangle1[0] = (int)faceIndexs[1];
									triangle1[1] = (int)faceIndexs[2];
									triangle1[2] = (int)faceIndexs[3];
									
									triangle2[0] = (int)faceIndexs[0];
									triangle2[1] = (int)faceIndexs[1];
									triangle2[2] = (int)faceIndexs[2];
									
									triangles.add(triangle1);
									triangleMaterial.add(material1);
									triangles.add(triangle2);
									triangleMaterial.add(material2);
	/*
									int[] triangle3 = new int[3];
									int[] triangle4 = new int[3];
	
									triangle3[0] = (int)faceIndexs[2];
									triangle3[1] = (int)faceIndexs[1];
									triangle3[2] = (int)faceIndexs[0];
									
									triangle4[0] = (int)faceIndexs[3];
									triangle4[1] = (int)faceIndexs[2];
									triangle4[2] = (int)faceIndexs[0];
									
									triangles.add(triangle3);
									triangleMaterial.add(material3);
									triangles.add(triangle4);
									triangleMaterial.add(material4);
									*/
								}
							}
						}
						
					}
				}
			}
			
			int count = 0;
			while(r.read() >= 0) {
				count++;
			}
			System.out.println("Overrun bits: " + count);
			
			double[][] minMax = {{Double.MAX_VALUE,Double.MIN_VALUE},{Double.MAX_VALUE,Double.MIN_VALUE},{Double.MAX_VALUE,Double.MIN_VALUE}};

			for(double[] pt : this.points) {
				for(int i=0;i<3;i++) {
					minMax[i][0] = Math.min(minMax[i][0], pt[i]);
					minMax[i][1] = Math.max(minMax[i][1], pt[i]);
				}
			}
			System.out.println("Min max");
			System.out.println(minMax[0][0] + " " + minMax[0][1]);
			System.out.println(minMax[1][0] + " " + minMax[1][1]);
			System.out.println(minMax[2][0] + " " + minMax[2][1]);

			int min = Integer.MAX_VALUE;
			int max = Integer.MIN_VALUE;
			
			for(int[] triangle : triangles) {
				for(int val : triangle) {
					min = Math.min(min, val);
					max = Math.max(max, val);
				}
			}
			
			System.out.println("Pt range: " + min  + " " + max);
			
			
			if(max >= points.size()) {
				System.out.println("Error: Triangle point lies outside of valid set.");
				System.out.println(max + " >= " + points.size());
				System.exit(1);
			}
			
			if(min < 0) {
				System.out.println("Error: Triangle point lies outside of valid set.");
				System.out.println(min + " < " + 0);
				System.exit(1);
			}
			
			//transform.combine(TransformMy3D.stretch(10,10,10));
			transform = TransformMy3D.rotateZ(Math.PI);
			transform = TransformMy3D.rotateY(Math.PI / 2).combineNew(transform);
			transform = TransformMy3D.stretch(3, 3, 3).combineNew(transform);
			transform = TransformMy3D.translate(0,-2,0).combineNew(transform);
			
		} catch(IOException e) {
			e.printStackTrace();
		}
	}
	
	public static Object[] getVals(DataInputStream reader, int format, PLYElement element) throws IOException {
	
		Object[] vals = new Object[element.properties.size()];
		
		int i = 0;
		for(PLYProperty property : element.properties) {
			
			if(property.isList()) {
				Integer count = (Integer)extractType(reader,property.countType,format);					
							
				if(count == null) {
					return null;
				}
				Object[] list = new Object[count];					
				
				boolean badList = false;
				for(int j=0;j<count;j++) {
					
					list[j] = extractType(reader,property.primaryType,format);						
					//list[j] = new Integer((Integer)list[j] + 2);
											
					//if((Integer)list[j] >= 346349) {
					//	System.out.println(Integer.toHexString((Integer)list[j]) + " " + Integer.toHexString(346349));
					//	badList = true;
					//}							
						
					if(list[j] == null) {
						return null;
					}
				}

				if(badList) {
					for(int k=0;k<count;k++) {
						list[k] = new Integer(100);
					}
				}
				
				vals[i] = list;

			} else {
				vals[i] = extractType(reader,property.primaryType, format);
								
				if(vals[i] == null) {
					return null;
				}
			}				
			i++;				
		}
		return vals;
	}
	
	public static String readElement(DataInputStream r) throws IOException {
		String ret = "";
		
		while(ret.length() == 0) {
			while(true) {
				int c = r.read();
				if(c < 0) {
					return null;
				}
				if(c == ' ' || c == '\r' || c == '\n') {
					break;
				} else {
					ret += (char)c;
				}
			}
			ret = ret.trim();
		}
		
		return ret;
	}
	
	public static Object extractType(DataInputStream r, String type, int format) throws IOException {
		
		if(format == Ply3D.FORMAT_ASCII) {
			if(type.equals("int8") || type.equals("uchar") || type.equals("uint8") || type.equals("short") || type.equals("int32")) {
				String str = readElement(r);
				if(str == null) {
					return null;
				} else {
					return new Integer(Integer.parseInt(str));
				}
			} else if(type.equals("float") || type.equals("float32")) {
				String str = readElement(r);
				if(str == null) {
					return null;
				} else {
					return new Double(Double.parseDouble(str));
				}
			} else {
				System.out.println("Unknown type [" + type + "]");
				return null;
			}
		} else {
			
			
			boolean littleEndian = (format == Ply3D.FORMAT_BINARY_LITTLE_ENDIAN);
			if(type.equals("int8") || type.equals("uchar") || type.equals("uint8")) {
				
				int byteVal = r.read();
								
				if(byteVal < 0) {
					return null;
				}
				return new Integer(byteVal & 0xFF);
			} else if(type.equals("short")) {
				return readShort(r,littleEndian);
			} else if(type.equals("int32")) {
				return readInt(r,littleEndian);
			} else if(type.equals("float") || type.equals("float32")) {
				Integer intVal = readInt(r,littleEndian);			
				
				//System.out.println(Integer.toHexString(intVal));
				
				if(intVal == null) {
					return null;
				}
				float val = Float.intBitsToFloat(intVal);
				return new Double(val);
			} else {
				System.out.println("Unknown type [" + type + "]");
				return null;
			}
		}
	}
	
	public static Integer readShort(DataInputStream reader, boolean littleEndian) throws IOException {
		int asInt = 0;
		
		int b1,b2;
		
		if(littleEndian) {
			b1 = reader.read();
			b2 = reader.read();
		} else {
			b2 = reader.read();
			b1 = reader.read();
		}
		
		if(b1 < 0 || b2 < 0) {
			return null;
		}
		
		asInt |= ((b1 << 0) & 0xFF);
		asInt |= ((b2 << 8) & 0xFF00);
		
		return new Integer(asInt);
	}
	
	public static Integer readInt(DataInputStream reader, boolean littleEndian) throws IOException {
		/*
		int asInt = 0;
		
		char[] bytes = new char[4];
		if(reader.read(bytes) != bytes.length) {
			return null;
		}
		
		for(int i=0;i<bytes.length;i++) {
			System.out.print(Integer.toHexString(bytes[i] & 0xFF) + " ");
		}
		System.out.println();
		
		if(littleEndian) {
			asInt |= (bytes[0] << 0);
			asInt |= (bytes[1] << 8);
			asInt |= (bytes[2] << 16);
			asInt |= (bytes[3] << 24);
		} else {
			asInt |= (bytes[3] << 0);
			asInt |= (bytes[2] << 8);
			asInt |= (bytes[1] << 16);
			asInt |= (bytes[0] << 24);
		}
				
		return new Integer(asInt);
		*/
		
		
		int asInt = 0;
		
		int b1,b2,b3,b4;
		
		if(littleEndian) {
			b1 = reader.read();
			b2 = reader.read();
			b3 = reader.read();
			b4 = reader.read();
		} else {
			b4 = reader.read();
			b3 = reader.read();
			b2 = reader.read();
			b1 = reader.read();
		}
		
		//System.out.print(Integer.toHexString(b4 & 0xFF) + " ");
		//System.out.print(Integer.toHexString(b3 & 0xFF) + " ");
		//System.out.print(Integer.toHexString(b2 & 0xFF) + " ");
		//System.out.print(Integer.toHexString(b1 & 0xFF) + " ");
		//System.out.println();
		
		if(b1 < 0 || b2 < 0 || b3 < 0 || b4 < 0) {
			return null;
		}
		
		//b1 = (b1 & 0xFF);
		//b2 = (b2 & 0xFF);
		//b3 = (b3 & 0xFF);
		//b4 = (b4 & 0xFF);
				
		asInt |= (b1 << 0);
		asInt |= (b2 << 8);
		asInt |= (b3 << 16);
		asInt |= (b4 << 24);
				
		return new Integer(asInt);
		
	}
	
	class PLYElement {
		public int count;
		
		public Vector<PLYProperty> properties = new Vector<PLYProperty>();
		public int type;
		
		public PLYElement(int type) {
			this.type = type;
		}	
		
		public String toString() {
			String ret = "{" + type + "|" + count + "}{";
			for(PLYProperty property : properties) {
				ret += property;
			}
			ret += "}";
			return ret;
		}
	}
	
	class PLYProperty {
		public String name;

		public String primaryType;
		public String countType = null;
		
		public PLYProperty(String name, String primaryType) {
			this.name = name;
			this.primaryType = primaryType;
		}
		
		public PLYProperty(String name, String primaryType, String countType) {
			this.name = name;
			this.primaryType = primaryType;
			this.countType = countType;			
		}
		
		public boolean isList() {
			return countType != null;
		}
		
		public String toString() {
			return "[" + name + "," + primaryType + "]";
		}
	}
/*
 ply
        format ascii 1.0           { ascii/binary, format version number }
        comment made by anonymous  { comments are keyword specified }
        comment this file is a cube
        element vertex 8           { define "vertex" element, 8 in file }
        property float32 x         { vertex contains float "x" coordinate }
        property float32 y         { y coordinate is also a vertex property }
        property float32 z         { z coordinate, too }
        element face 6             { there are 6 "face" elements in the file }
        property list uint8 int32 vertex_index 
                                   { "vertex_indices" is a list of ints }
        end_header                 { delimits the end of the header }
        0 0 0                      { start of vertex list }
        0 0 1
        0 1 1
        0 1 0
        1 0 0
        1 0 1
        1 1 1
        1 1 0
        4 0 1 2 3                  { start of face list }
        4 7 6 5 4
        4 0 4 5 1
        4 1 5 6 2
        4 2 6 7 3
        4 3 7 4 0

 */
}
