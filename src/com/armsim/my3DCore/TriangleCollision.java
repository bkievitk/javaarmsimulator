package com.armsim.my3DCore;

public class TriangleCollision {

	/* Triangle/triangle intersection test routine,
	 * by Tomas Moller, 1997.
	 * See article "A Fast Triangle-Triangle Intersection Test",
	 * Journal of Graphics Tools, 2(2), 1997
	 * updated: 2001-06-20 (added line of intersection)
	 *
	 * int tri_tri_intersect(float V0[3],float V1[3],float V2[3],
	 *                       float U0[3],float U1[3],float U2[3])
	 *
	 * parameters: vertices of triangle 1: V0,V1,V2
	 *             vertices of triangle 2: U0,U1,U2
	 * result    : returns 1 if the triangles intersect, otherwise 0
	 *
	 * Here is a version withouts divisions (a little faster)
	 * int NoDivTriTriIsect(float V0[3],float V1[3],float V2[3],
	 *                      float U0[3],float U1[3],float U2[3]);
	 * 
	 * This version computes the line of intersection as well (if they are not coplanar):
	 * int tri_tri_intersect_with_isectline(float V0[3],float V1[3],float V2[3], 
	 *				        float U0[3],float U1[3],float U2[3],int *coplanar,
	 *				        float isectpt1[3],float isectpt2[3]);
	 * coplanar returns whether the tris are coplanar
	 * isectpt1, isectpt2 are the endpoints of the line of intersection
	 */

	public static final boolean USE_EPSILON_TEST = true;
	public static final double EPSILON = 0.000001;

	/* some macros */
	public static void cross(double[] dest, double[] v1, double[] v2) {
		dest[0]=v1[1]*v2[2]-v1[2]*v2[1];
		dest[1]=v1[2]*v2[0]-v1[0]*v2[2];
		dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
	}

	public static double dot(double[] v1, double[] v2) {
		return (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]);
	}

	public static void sub(double[] dest,double[] v1,double[] v2) {
		dest[0]=v1[0]-v2[0]; 
		dest[1]=v1[1]-v2[1]; 
		dest[2]=v1[2]-v2[2]; 
	}

	public static void add(double[] dest,double[] v1,double[] v2) {
		dest[0]=v1[0]+v2[0]; 
		dest[1]=v1[1]+v2[1]; 
		dest[2]=v1[2]+v2[2]; 
	}

	public static void mult(double[] dest,double[] v,double factor) {
		dest[0]=factor*v[0];
		dest[1]=factor*v[1];
		dest[2]=factor*v[2];
	}

	public static void set(double[] dest, double[] src) {
		dest[0]=src[0];
		dest[1]=src[1];
		dest[2]=src[2]; 
	}

	public static void sort(double[] val) {
		if(val[0] > val[1]) {
			double tmp = val[0];
			val[0] = val[1];
			val[1] = tmp;
		}
	}

	public static void isect(double VV0, double VV1, double VV2, double D0, double D1, double D2, double[] isect) {
		isect[0] = VV0+(VV1-VV0)*D0/(D0-D1);
		isect[1] = VV0+(VV2-VV0)*D0/(D0-D2);
	}

	public static int compute_intervals(
			double VV0,double VV1,double VV2,double D0,double D1,double D2,double D0D1,double D0D2, double[] isect,
			double[] N1, double[] V0, double[] V1, double[] V2, double[] U0, double[] U1, double[] U2
	) {

		if(D0D1>0.0f)                                         
		{                                                     
			/* here we know that D0D2<=0.0 */                   
			/* that is D0, D1 are on the same side, D2 on the other or on the plane */ 
			isect(VV2,VV0,VV1,D2,D0,D1, isect);   
			return 0;
		}                                                     
		else if(D0D2>0.0f)                                    
		{                                                     
			/* here we know that d0d1<=0.0 */                   
			isect(VV1,VV0,VV2,D1,D0,D2, isect);  
			return 0;          
		}                                                     
		else if(D1*D2>0.0f || D0!=0.0f)                       
		{                                                     
			/* here we know that d0d1<=0.0 or that D0!=0.0 */   
			isect(VV0,VV1,VV2,D0,D1,D2, isect);  
			return 0;          
		}                                                     
		else if(D1!=0.0f)                                     
		{                                                     
			isect(VV1,VV0,VV2,D1,D0,D2, isect);   
			return 0;       
		}                                                     
		else if(D2!=0.0f)                                     
		{                                                     
			isect(VV2,VV0,VV1,D2,D0,D1, isect);   
			return 0;       
		}                                                     
		else                                                  
		{                                                
			/* triangles are coplanar */                        
			if(coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2)) {
				return 1;
			} else {
				return -1;
			}
		}
	}

	/* this edge to edge test is based on Franlin Antonio's gem:
	   "Faster Line Segment Intersection", in Graphics Gems III,
	   pp. 199-202 */ 
	public static boolean edge_edge_test(double[] V0, double[] U0, double[] U1, double Ax, double Ay, int i0, int i1) {
		double Bx = U0[i0]-U1[i0];                                   
		double By = U0[i1]-U1[i1];                                   
		double Cx = V0[i0]-U0[i0];                                   
		double Cy = V0[i1]-U0[i1];                                   
		double f = Ay*Bx-Ax*By;                                      
		double d = By*Cx-Bx*Cy;                                      
		if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f)) {                                                   
			double e=Ax*Cy-Ay*Cx;                                    
			if(f>0) {                                                 
				if(e>=0 && e<=f) {
					return true;
				}
			}                                                 
			else {                                                 
				if(e<=0 && e>=f) {
					return true;  
				}
			}                                                 
		}    
		return false;
	}


	public static boolean edge_against_tri_edges(double[] V0,double[] V1,double[] U0,double[] U1,double[] U2, int i0, int i1) {

		double Ax=V1[i0]-V0[i0];                            
		double Ay=V1[i1]-V0[i1];                            
		/* test edge U0,U1 against V0,V1 */          
		if(edge_edge_test(V0,U0,U1,Ax,Ay, i0, i1)) {
			return true;
		}

		/* test edge U1,U2 against V0,V1 */          
		if(edge_edge_test(V0,U1,U2,Ax,Ay, i0, i1)) {
			return true;
		}

		/* test edge U2,U1 against V0,V1 */          
		if(edge_edge_test(V0,U2,U0,Ax,Ay, i0, i1)) {
			return true;
		}

		return false;
	}

	public static boolean point_in_tri(double[] V0, double[] U0, double[] U1, double[] U2, int i0, int i1) {

		double a,b,c,d0,d1,d2;                     
		/* is T1 completly inside T2? */          
		/* check if V0 is inside tri(U0,U1,U2) */ 
		a=U1[i1]-U0[i1];                          
		b=-(U1[i0]-U0[i0]);                       
		c=-a*U0[i0]-b*U0[i1];                     
		d0=a*V0[i0]+b*V0[i1]+c;                   

		a=U2[i1]-U1[i1];                          
		b=-(U2[i0]-U1[i0]);                       
		c=-a*U1[i0]-b*U1[i1];                     
		d1=a*V0[i0]+b*V0[i1]+c;                   

		a=U0[i1]-U2[i1];                          
		b=-(U0[i0]-U2[i0]);                       
		c=-a*U2[i0]-b*U2[i1];                     
		d2=a*V0[i0]+b*V0[i1]+c;  

		if(d0*d1>0.0)                             
		{                                         
			if(d0*d2>0.0) {
				return true;                 
			}
		}                
		return false;
	}

	public static boolean coplanar_tri_tri(double[] N,double[] V0,double[] V1,double[] V2, double[] U0,double[] U1,double[] U2)
	{
		double[] A = new double[3];
		short i0,i1;
		/* first project onto an axis-aligned plane, that maximizes the area */
		/* of the triangles, compute indices: i0,i1. */
		A[0]=Math.abs(N[0]);
		A[1]=Math.abs(N[1]);
		A[2]=Math.abs(N[2]);
		if(A[0]>A[1])
		{
			if(A[0]>A[2])  
			{
				i0=1;      /* A[0] is greatest */
				i1=2;
			}
			else
			{
				i0=0;      /* A[2] is greatest */
				i1=1;
			}
		}
		else   /* A[0]<=A[1] */
		{
			if(A[2]>A[1])
			{
				i0=0;      /* A[2] is greatest */
				i1=1;                                           
			}
			else
			{
				i0=0;      /* A[1] is greatest */
				i1=2;
			}
		}               

		/* test all edges of triangle 1 against the edges of triangle 2 */
		if(edge_against_tri_edges(V0,V1,U0,U1,U2,i0,i1)) {
			return true;
		}
		if(edge_against_tri_edges(V1,V2,U0,U1,U2,i0,i1)) {
			return true;
		}
		if( edge_against_tri_edges(V2,V0,U0,U1,U2,i0,i1)) {
			return true;
		}

		/* finally, test if tri1 is totally contained in tri2 or vice versa */
		if( point_in_tri(V0,U0,U1,U2,i0,i1)) {
			return true;
		}
		if(point_in_tri(U0,V0,V1,V2,i0,i1)) {
			return true;
		}

		return false;
	}


	public static boolean tri_tri_intersect(double[] V0,double[] V1,double[] V2, double[] U0,double[] U1,double[] U2) {
		double[] E1 = new double[3];
		double[] E2 = new double[3];
		double[] N1 = new double[3];
		double[] N2 = new double[3];
		double d1,d2;
		double du0,du1,du2,dv0,dv1,dv2;
		double[] D = new double[3];
		double[] isect1 = new double[2];
		double[] isect2 = new double[2];
		double du0du1,du0du2,dv0dv1,dv0dv2;
		short index;
		double vp0,vp1,vp2;
		double up0,up1,up2;
		double b,c,max;

		/* compute plane equation of triangle(V0,V1,V2) */
		sub(E1,V1,V0);
		sub(E2,V2,V0);
		cross(N1,E1,E2);
		d1=-dot(N1,V0);
		/* plane equation 1: N1.X+d1=0 */

		/* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
		du0=dot(N1,U0)+d1;
		du1=dot(N1,U1)+d1;
		du2=dot(N1,U2)+d1;

		/* coplanarity robustness check */
		if(USE_EPSILON_TEST) {
			if(Math.abs(du0)<EPSILON) { du0=0.0; }
			if(Math.abs(du1)<EPSILON) { du1=0.0; }
			if(Math.abs(du2)<EPSILON) { du2=0.0; }
		}
		du0du1=du0*du1;
		du0du2=du0*du2;

		if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
			return false;                    /* no intersection occurs */

		/* compute plane of triangle (U0,U1,U2) */
		sub(E1,U1,U0);
		sub(E2,U2,U0);
		cross(N2,E1,E2);
		d2=-dot(N2,U0);
		/* plane equation 2: N2.X+d2=0 */

		/* put V0,V1,V2 into plane equation 2 */
		dv0=dot(N2,V0)+d2;
		dv1=dot(N2,V1)+d2;
		dv2=dot(N2,V2)+d2;

		if(USE_EPSILON_TEST) {
			if(Math.abs(dv0)<EPSILON) {dv0=0.0;}
			if(Math.abs(dv1)<EPSILON) {dv1=0.0;}
			if(Math.abs(dv2)<EPSILON) {dv2=0.0;}
		}

		dv0dv1=dv0*dv1;
		dv0dv2=dv0*dv2;

		if(dv0dv1>0.0f && dv0dv2>0.0f) { /* same sign on all of them + not equal 0 ? */
			return false;                    /* no intersection occurs */
		}

		/* compute direction of intersection line */
		cross(D,N1,N2);

		/* compute and index to the largest component of D */
		max=Math.abs(D[0]);
		index=0;
		b=Math.abs(D[1]);
		c=Math.abs(D[2]);
		if(b>max) {
			max=b;
			index=1;
		}
		if(c>max) {
			max=c;
			index=2;
		}

		/* this is the simplified projection onto L*/
		vp0=V0[index];
		vp1=V1[index];
		vp2=V2[index];

		up0=U0[index];
		up1=U1[index];
		up2=U2[index];

		/* compute interval for triangle 1 */
		int result = compute_intervals(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,isect1,N1,V0,V1,V2,U0,U1,U2);
		if(result == -1) {
			return false;
		} else if(result == 1) {
			return true;
		}
				
		/* compute interval for triangle 2 */
		result = compute_intervals(up0,up1,up2,du0,du1,du2,du0du1,du0du2,isect2,N1,V0,V1,V2,U0,U1,U2);
		if(result == -1) {
			return false;
		} else if(result == 1) {
			return true;
		}

		sort(isect1);
		sort(isect2);

		if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) {
			return false;
		}
		return true;
	}


	public static int newcompute_intervals(double VV0,double VV1,double VV2,double D0,double D1,double D2,double D0D1,double D0D2,double[] vals,
			double[] N1, double[] V0, double[] V1, double[] V2, double[] U0, double[] U1, double[] U2) {

		if(D0D1>0.0f) 
		{ 
			/* here we know that D0D2<=0.0 */ 
			/* that is D0, D1 are on the same side, D2 on the other or on the plane */ 
			vals[0]=VV2; 
			vals[1]=(VV0-VV2)*D2; 
			vals[2]=(VV1-VV2)*D2; 
			vals[3]=D2-D0; 
			vals[4]=D2-D1; 
			return 0;
		} 
		else if(D0D2>0.0f)
		{ 
			/* here we know that d0d1<=0.0 */ 
			vals[0]=VV1; 
			vals[1]=(VV0-VV1)*D1; 
			vals[2]=(VV2-VV1)*D1; 
			vals[3]=D1-D0; 
			vals[4]=D1-D2; 
			return 0;
		} 
		else if(D1*D2>0.0f || D0!=0.0f) 
		{ 
			/* here we know that d0d1<=0.0 or that D0!=0.0 */ 
			vals[0]=VV0; 
			vals[1]=(VV1-VV0)*D0; 
			vals[2]=(VV2-VV0)*D0; 
			vals[3]=D0-D1; 
			vals[4]=D0-D2; 
			return 0;
		} 
		else if(D1!=0.0f) 
		{ 
			vals[0]=VV1; 
			vals[1]=(VV0-VV1)*D1; 
			vals[2]=(VV2-VV1)*D1; 
			vals[3]=D1-D0; 
			vals[4]=D1-D2; 
			return 0;
		} 
		else if(D2!=0.0f) 
		{ 
			vals[0]=VV2; 
			vals[1]=(VV0-VV2)*D2; 
			vals[2]=(VV1-VV2)*D2; 
			vals[3]=D2-D0; 
			vals[4]=D2-D1; 
			return 0;
		} 
		else 
		{ 
			/* triangles are coplanar */ 
			if(coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2)) {
				return 1;
			} else {
				return -1;
			}
		}
	}

	public static boolean NoDivTriTriIsect(double[] V0,double[] V1,double[] V2,double[] U0,double[] U1,double[] U2) {
		double[] E1 = new double[3];
		double[] E2 = new double[3];
		double[] N1 = new double[3];
		double[] N2 = new double[3];
		double d1,d2;
		double du0,du1,du2,dv0,dv1,dv2;
		double[] D = new double[3];
		double[] isect1 = new double[2];
		double[] isect2 = new double[2];
		double du0du1,du0du2,dv0dv1,dv0dv2;
		short index;
		double vp0,vp1,vp2;
		double up0,up1,up2;
		double bb,cc,max;
		double a = 0,b = 0,c = 0, x0 = 0,x1 = 0;
		double d = 0,e = 0,f = 0, y0 = 0,y1 = 0;
		double xx,yy,xxyy,tmp;

		/* compute plane equation of triangle(V0,V1,V2) */
		sub(E1,V1,V0);
		sub(E2,V2,V0);
		cross(N1,E1,E2);
		d1=-dot(N1,V0);
		/* plane equation 1: N1.X+d1=0 */

		/* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
		du0=dot(N1,U0)+d1;
		du1=dot(N1,U1)+d1;
		du2=dot(N1,U2)+d1;

		/* coplanarity robustness check */
		if(USE_EPSILON_TEST) {
			if(Math.abs(du0)<EPSILON) {du0=0.0;}
			if(Math.abs(du1)<EPSILON) {du1=0.0;}
			if(Math.abs(du2)<EPSILON) {du2=0.0;}
		}
		du0du1=du0*du1;
		du0du2=du0*du2;

		if(du0du1>0.0f && du0du2>0.0f) { /* same sign on all of them + not equal 0 ? */
			return false;                    /* no intersection occurs */
		}

		/* compute plane of triangle (U0,U1,U2) */
		sub(E1,U1,U0);
		sub(E2,U2,U0);
		cross(N2,E1,E2);
		d2=-dot(N2,U0);
		/* plane equation 2: N2.X+d2=0 */

		/* put V0,V1,V2 into plane equation 2 */
		dv0=dot(N2,V0)+d2;
		dv1=dot(N2,V1)+d2;
		dv2=dot(N2,V2)+d2;

		if( USE_EPSILON_TEST) {
			if(Math.abs(dv0)<EPSILON) {dv0=0.0;}
			if(Math.abs(dv1)<EPSILON) {dv1=0.0;}
			if(Math.abs(dv2)<EPSILON) {dv2=0.0;}
		}

		dv0dv1=dv0*dv1;
		dv0dv2=dv0*dv2;

		if(dv0dv1>0.0f && dv0dv2>0.0f) { /* same sign on all of them + not equal 0 ? */
			return false;                    /* no intersection occurs */
		}

		/* compute direction of intersection line */
		cross(D,N1,N2);

		/* compute and index to the largest component of D */
		max=(float)Math.abs(D[0]);
		index=0;
		bb=(float)Math.abs(D[1]);
		cc=(float)Math.abs(D[2]);
		if(bb>max) {
			max=bb;
			index=1;
		}
		if(cc>max) {
			max=cc;
			index=2;
		}

		/* this is the simplified projection onto L*/
		vp0=V0[index];
		vp1=V1[index];
		vp2=V2[index];

		up0=U0[index];
		up1=U1[index];
		up2=U2[index];

		/* compute interval for triangle 1 */
		double[] vals = new double[5];
		int val = newcompute_intervals(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,vals,N1, V0, V1,V2, U0, U1,  U2);
		if(val == 1) {
			return true;
		} else if(val == -1) {
			return false;
		}
		a = vals[0];
		b = vals[1];
		c = vals[2];
		x0 = vals[3];
		x1 = vals[4];
		
		/* compute interval for triangle 2 */
		val = newcompute_intervals(up0,up1,up2,du0,du1,du2,du0du1,du0du2,vals,N1, V0, V1,V2, U0, U1,  U2);
		if(val == 1) {
			return true;
		} else if(val == -1) {
			return false;
		}
		d = vals[0];
		e = vals[1];
		f = vals[2];
		y0 = vals[3];
		y1 = vals[4];
		
		xx=x0*x1;
		yy=y0*y1;
		xxyy=xx*yy;

		tmp=a*xxyy;
		isect1[0]=tmp+b*x1*yy;
		isect1[1]=tmp+c*x0*yy;

		tmp=d*xxyy;
		isect2[0]=tmp+e*xx*y1;
		isect2[1]=tmp+f*xx*y0;

		sort(isect1);
		sort(isect2);

		if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) {
			return false;
		}
		return true;
	}

	// sort so that a<=b 
	public static boolean sort2(double[] val) {
		if(val[0] > val[1]) {
			double tmp = val[0];
			val[0] = val[1];
			val[1] = tmp;
			return true;
		}
		return false;
	}

	public static void isect2(double[] VTX0,double[] VTX1,double[] VTX2,double VV0,double VV1,double VV2, double D0,double D1,double D2,double[] isect,double[] isectpoint0,double[] isectpoint1) {
		double tmp=D0/(D0-D1);          
		double[] diff = new double[3];
		isect[0]=VV0+(VV1-VV0)*tmp;         
		sub(diff,VTX1,VTX0);              
		mult(diff,diff,tmp);               
		add(isectpoint0,diff,VTX0);        
		tmp=D0/(D0-D2);                    
		isect[1]=VV0+(VV2-VV0)*tmp;          
		sub(diff,VTX2,VTX0);                   
		mult(diff,diff,tmp);                 
		add(isectpoint1,VTX0,diff);          
	}

	/*
	#if 0
	#define ISECT2(VTX0,VTX1,VTX2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1) \
	              tmp=D0/(D0-D1);                    \
	              isect0=VV0+(VV1-VV0)*tmp;          \
		      SUB(diff,VTX1,VTX0);               \
		      MULT(diff,diff,tmp);               \
	              ADD(isectpoint0,diff,VTX0);        \ 
	              tmp=D0/(D0-D2);                    
	/*              isect1=VV0+(VV2-VV0)*tmp;          \ */
	/*              SUB(diff,VTX2,VTX0);               \     */
	/*              MULT(diff,diff,tmp);               \   */
	/*              ADD(isectpoint1,VTX0,diff);           */
	//#endif

	public static boolean compute_intervals_isectline(double[] VERT0,double[] VERT1,double[] VERT2,
			double VV0,double VV1,double VV2,double D0,double D1,double D2,
			double D0D1,double D0D2,double[] isect,
			double[] isectpoint0,double[] isectpoint1) {
		if(D0D1>0.0f)                                        
		{                                                    
			/* here we know that D0D2<=0.0 */                  
			/* that is D0, D1 are on the same side, D2 on the other or on the plane */
			isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect,isectpoint0,isectpoint1);
		} 
		else if(D0D2>0.0f)                                   
		{                                                   
			/* here we know that d0d1<=0.0 */             
			isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect,isectpoint0,isectpoint1);
		}                                                  
		else if(D1*D2>0.0f || D0!=0.0f)   
		{                                   
			/* here we know that d0d1<=0.0 or that D0!=0.0 */
			isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,isect,isectpoint0,isectpoint1);   
		}                                                  
		else if(D1!=0.0f)                                  
		{                                               
			isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect,isectpoint0,isectpoint1); 
		}                                         
		else if(D2!=0.0f)                                  
		{                                                   
			isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect,isectpoint0,isectpoint1);     
		}                                                 
		else                                               
		{                                                   
			/* triangles are coplanar */    
			return true;
		}
		return false;
	}
/*
	public static boolean compute_intervals_isectline(double[] VERT0,double[] VERT1,double[] VERT2,double VV0,double VV1,double VV2,double D0,double D1,double D2,double D0D1,double D0D2,double isect0,double isect1,double[] isectpoint0,double[] isectpoint1) {
		if(D0D1>0.0f) {                                                     
			// here we know that D0D2<=0.0                    
			// that is D0, D1 are on the same side, D2 on the other or on the plane  
			isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);          
		}                                                     
	#if 0
	  else if(D0D2>0.0f)                                    
	  {                                                     
	    // here we know that d0d1<=0.0                  
	    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);         
	  }                                                     
	  else if(D1*D2>0.0f || D0!=0.0f)                       
	  {                                                     
	    // here we know that d0d1<=0.0 or that D0!=0.0    
	    isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,&isect0,&isect1,isectpoint0,isectpoint1);          
	  }                                                     
	  else if(D1!=0.0f)                                     
	  {                                                     
	    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          
	  }                                                     
	  else if(D2!=0.0f)                                     
	  {                                                     
	    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          
	  }                                                     
	  else                                                  
	  {                                                     
	    // triangles are coplanar                       
	    coplanar=1;                                         
	    return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      
	  }
	#endif
		 
		return true;
	}
*/

	public static boolean tri_tri_intersect_with_isectline(double[] V0,double[] V1,double[] V2,
			double[] U0,double[] U1,double[] U2,boolean[] coplanar,
			double[] isectpt1,double[] isectpt2)
	{
		double[] E1 = new double[3];
		double[] E2 = new double[3];
		double[] N1 = new double[3];
		double[] N2 = new double[3];
		double d1,d2;
		double du0,du1,du2,dv0,dv1,dv2;
		double[] D = new double[3];
		double[] isect1 = new double[2];
		double[] isect2 = new double[2];
		double[] isectpointA1 = new double[3];
		double[] isectpointA2 = new double[3];
		double[] isectpointB1 = new double[3];
		double[] isectpointB2 = new double[3];
		double du0du1,du0du2,dv0dv1,dv0dv2;
		short index;
		double vp0,vp1,vp2;
		double up0,up1,up2;
		double b,c,max;
		//double tmp;
		//double[] diff = new double[3];
		boolean smallest1, smallest2;

		/* compute plane equation of triangle(V0,V1,V2) */
		sub(E1,V1,V0);
		sub(E2,V2,V0);
		cross(N1,E1,E2);
		d1=-dot(N1,V0);
		/* plane equation 1: N1.X+d1=0 */

		/* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
		du0=dot(N1,U0)+d1;
		du1=dot(N1,U1)+d1;
		du2=dot(N1,U2)+d1;

		/* coplanarity robustness check */
		if( USE_EPSILON_TEST) {
			if(Math.abs(du0)<EPSILON) du0=0.0;
			if(Math.abs(du1)<EPSILON) du1=0.0;
			if(Math.abs(du2)<EPSILON) du2=0.0;
		}
		du0du1=du0*du1;
		du0du2=du0*du2;

		if(du0du1>0.0f && du0du2>0.0f) { /* same sign on all of them + not equal 0 ? */
			return false;                    /* no intersection occurs */
		}

		/* compute plane of triangle (U0,U1,U2) */
		sub(E1,U1,U0);
		sub(E2,U2,U0);
		cross(N2,E1,E2);
		d2=-dot(N2,U0);
		/* plane equation 2: N2.X+d2=0 */

		/* put V0,V1,V2 into plane equation 2 */
		dv0=dot(N2,V0)+d2;
		dv1=dot(N2,V1)+d2;
		dv2=dot(N2,V2)+d2;

		if (USE_EPSILON_TEST) {
			if(Math.abs(dv0)<EPSILON) dv0=0.0;
			if(Math.abs(dv1)<EPSILON) dv1=0.0;
			if(Math.abs(dv2)<EPSILON) dv2=0.0;
		}

		dv0dv1=dv0*dv1;
		dv0dv2=dv0*dv2;

		if(dv0dv1>0.0f && dv0dv2>0.0f) { /* same sign on all of them + not equal 0 ? */
			return false;                    /* no intersection occurs */
		}

		/* compute direction of intersection line */
		cross(D,N1,N2);

		/* compute and index to the largest component of D */
		max=Math.abs(D[0]);
		index=0;
		b=Math.abs(D[1]);
		c=Math.abs(D[2]);
		if(b>max) {
			max=b;
			index=1;
		}

		if(c>max) {
			max=c;
			index=2;
		}

		/* this is the simplified projection onto L*/
		vp0=V0[index];
		vp1=V1[index];
		vp2=V2[index];

		up0=U0[index];
		up1=U1[index];
		up2=U2[index];

		/* compute interval for triangle 1 */
		coplanar[0] = compute_intervals_isectline(V0,V1,V2,vp0,vp1,vp2,dv0,dv1,dv2,
				dv0dv1,dv0dv2,isect1,isectpointA1,isectpointA2);
		if(coplanar[0]) {
			return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);     
		}

		/* compute interval for triangle 2 */
		compute_intervals_isectline(U0,U1,U2,up0,up1,up2,du0,du1,du2,
				du0du1,du0du2,isect2,isectpointB1,isectpointB2);

		smallest1 = sort2(isect1);
		smallest2 = sort2(isect2);

		if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) { return false; }

		/* at this point, we know that the triangles intersect */

		if(isect2[0]<isect1[0])
		{
			if(!smallest1) { set(isectpt1,isectpointA1); }
			else { set(isectpt1,isectpointA2); }

			if(isect2[1]<isect1[1])
			{
				if(!smallest2) { set(isectpt2,isectpointB2); }
				else { set(isectpt2,isectpointB1); }
			}
			else
			{
				if(!smallest1) { set(isectpt2,isectpointA2); }
				else { set(isectpt2,isectpointA1); }
			}
		}
		else
		{
			if(!smallest2) { set(isectpt1,isectpointB1); }
			else { set(isectpt1,isectpointB2); }

			if(isect2[1]>isect1[1])
			{
				if(!smallest1) { set(isectpt2,isectpointA2); }
				else { set(isectpt2,isectpointA1); }      
			}
			else
			{
				if(!smallest2) { set(isectpt2,isectpointB2); }
				else { set(isectpt2,isectpointB1); } 
			}
		}
		return true;
	}




}
