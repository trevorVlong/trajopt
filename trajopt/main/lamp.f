
      subroutine lamp( x1, r1, x2, r2,  xf, rf, rcore,
     &                 ug1, vg1, ug2, vg2, 
     &                 us1, vs1, us2, vs2 )
C-----------------------------------------------------------------------

C     Computes the velocities at xf,rf induced by a vortex + source 
C     sheet "lampshade"  extending from x1,r1 to x2,r2.  

c      Vortex sheet density = gamma  (positive counterclockwise)
C      Source sheet density = sigma

C     Both densities are assumed to be linear in meridional 
C     arc length over the panel.


C  Input:
C  ------
C     x1,r1   x,r of one   lampshade edge
c     x2,r2   x,r of other lampshade edge
c     xf,rf   x,r of the field point

C  Output: 
C  -------
c     ug1,vg1   x,r velocities at xf,rf for unit gamma at x1,r1
c     ug2,vg2   x,r velocities at xf,rf for unit gamma at x2,r2
c     us1,vs1   x,r velocities at xf,rf for unit sigma at x1,r1
c     us2,vs2   x,r velocities at xf,rf for unit sigma at x2,r2

c     Total x,r velocities for given endpoint sheet densities are:

c       u = ug1*gamma1 + ug2*gamma2 + us1*sigma1 + us2*sigma2
c       v = vg1*gamma1 + vg2*gamma2 + vs1*sigma1 + vs2*sigma2

c-----------------------------------------------------------------------

c     Untegrates point vortex/source ring velocities over 
C     the lampshade using a Romberg sequence applied to the 
C     simple midpoint rule:

C        |       *       |  ->  I1  -   I21  -   I321  -   I4321
C                                   /        /         /     .
C        |   *       *   |  ->  I2  -   I32  -   I432        .
C                                   /        /     .       8th order
C        | *   *   *   * |  ->  I3  -   I43        .       
C                                   /    .       6th order
C        |* * * * * * * *|  ->  I4       .        
C                .               .      4th order
C                .               .      
C                               2nd order
C               etc             

C  The first column I1,I2... are the 2nd-order integral approximations
C  computed with the stock midpoint rule.  The subsequent columns
C  are the Richardson extrapolations to higher-order accuracy.


C  Algorithm for four Romberg stages:

C    I21 = (4*I2 - I1) / 3           | extrapolation of 2nd-order results
C    I32 = (4*I3 - I2) / 3           |
C    I43 = (4*I4 - I3) / 3           |
C 
C    I321 = (16*I32 - I21) / 15      | extrapolation of 4th-order results
C    I432 = (16*I43 - I32) / 15      |

C    I4321 = (64*I432 - I321) / 63   | extrapolation of 6th-order results


C  Quantities stored in the nromx arrays after each irom stage:

c    irom  =   1      2       3       4     
C            ----   ----    -----   ------
C    u(1)  =  I1     I21     I321    I4321  
C    u(2)  =         I2      I32     I432   
C    u(3)  =                 I3      I43    
C    u(4)  =                         I4      

C-----------------------------------------------------------------------

C---- nromx = max number of Romberg stages
C-      This limits the integration resolution and cost.
C-      The influence of this panel on a field point 
C-      which is closer than  O(panel_length)/2**nromx 
C-      will not be accurately represented.
      parameter (nromx=10)

      dimension ug1i(nromx), vg1i(nromx),
     &          ug2i(nromx), vg2i(nromx),
     &          us1i(nromx), vs1i(nromx),
     &          us2i(nromx), vs2i(nromx)

      parameter (pi = 3.14159265358979323846264338327950280)

c---- romberg convergence tolerance
      data romtol / 1.0e-6 /
ccc      data romtol / 1.0e-12 /

c---- reference length for convergence tolerance 
      refl = 0.5*(r1 + r2)

c---- evaluate integrals over  0..t..1  on increasingly fine grids
      do 100 irom=1, nromx
        nt = 2**irom / 2

        ug1i(irom) = 0.
        vg1i(irom) = 0.
        ug2i(irom) = 0.
        vg2i(irom) = 0.
        us1i(irom) = 0.
        vs1i(irom) = 0.
        us2i(irom) = 0.
        vs2i(irom) = 0.

c------ visit the midpoints of each of the nt intervals
        do 10 it=1, nt
          t = (float(it) - 0.5) / float(nt)
          tb = 1.0 - t

          dt = 1.0/float(nt)

          xt = x1*tb + x2*t
          rt = r1*tb + r2*t

c-------- get induced velocities for vortex,source ring at xt,rt
          call ring( xt, rt, xf, rf, rcore, ugt, vgt, ust, vst )

c-------- accumulate the separate unit-gamma, unit-sigma integrals
          ug1i(irom) = ug1i(irom) + dt*ugt*tb
          vg1i(irom) = vg1i(irom) + dt*vgt*tb
          ug2i(irom) = ug2i(irom) + dt*ugt*t
          vg2i(irom) = vg2i(irom) + dt*vgt*t
          us1i(irom) = us1i(irom) + dt*ust*tb
          vs1i(irom) = vs1i(irom) + dt*vst*tb
          us2i(irom) = us2i(irom) + dt*ust*t
          vs2i(irom) = vs2i(irom) + dt*vst*t
 10     continue

c------ romberg sequence using all previous grid results
        do krom = irom, 2, -1
c-------- weight needed to cancel lowest-order error terms in krom level
          w = 2.0 ** (2*(irom-krom+1))

c-------- put richardson extrapolation for krom level into krom-1 level
          ug1i(krom-1) = (w*ug1i(krom) - ug1i(krom-1)) / (w-1.0)
          vg1i(krom-1) = (w*vg1i(krom) - vg1i(krom-1)) / (w-1.0)
          ug2i(krom-1) = (w*ug2i(krom) - ug2i(krom-1)) / (w-1.0)
          vg2i(krom-1) = (w*vg2i(krom) - vg2i(krom-1)) / (w-1.0)
          us1i(krom-1) = (w*us1i(krom) - us1i(krom-1)) / (w-1.0)
          vs1i(krom-1) = (w*vs1i(krom) - vs1i(krom-1)) / (w-1.0)
          us2i(krom-1) = (w*us2i(krom) - us2i(krom-1)) / (w-1.0)
          vs2i(krom-1) = (w*vs2i(krom) - vs2i(krom-1)) / (w-1.0)
        enddo

        if(irom.gt.1) then
c------- compare the best-current and best-previous integrals
         errug1 = ug1i(1) - ug1i(2)
         errvg1 = vg1i(1) - vg1i(2)
         errug2 = ug2i(1) - ug2i(2)
         errvg2 = vg2i(1) - vg2i(2)
         errus1 = us1i(1) - us1i(2)
         errvs1 = vs1i(1) - vs1i(2)
         errus2 = us2i(1) - us2i(2)
         errvs2 = vs2i(1) - vs2i(2)

         err = max(
     &     abs(errug1),
     &     abs(errvg1),
     &     abs(errug2),
     &     abs(errvg2),
     &     abs(errus1),
     &     abs(errvs1),
     &     abs(errus2),
     &     abs(errvs2) )
         
c         write(13,1200) 
c     &     abs(errug1),
c     &     abs(errvg1),
c     &     abs(errug2),
c     &     abs(errvg2),
c     &     abs(errus1),
c     &     abs(errvs1),
c     &     abs(errus2),
c     &     abs(errvs2)
c 1200    format(8e16.9)

         if(err*refl .lt. romtol) go to 101
        endif
 100  continue
      write(*,*) 'lamp: romberg convergence failed.  error =', err

 101  continue

ccc      write(*,*) irom, err

c---- return best final results
      delsq = (x1-x2)**2 + (r1-r2)**2
      dels = sqrt(delsq)

      ug1 = ug1i(1)*dels
      vg1 = vg1i(1)*dels
      ug2 = ug2i(1)*dels
      vg2 = vg2i(1)*dels
      us1 = us1i(1)*dels
      vs1 = vs1i(1)*dels
      us2 = us2i(1)*dels
      vs2 = vs2i(1)*dels

      return
      end ! lamp



      subroutine lampc( x1, r1, x2, r2, rcore,
     &                  ug1, vg1, ug2, vg2, 
     &                  us1, vs1, us2, vs2 )
C---------------------------------------------------------
C     Same as lamp, but the field point is assumed 
C     to be at the lampshade-panel midpoint.

C     The 1/r and log(r) singularities in the integrands 
C     are removed from the numerical integration,
C     and are computed analytically.

C     The induced velocites returned by this routine 
C     are the average of the two values on each side 
C     of the sheet.  The sheet jumps are not included.
C---------------------------------------------------------

C---- max number of Romberg integration stages
      parameter (nromx=9)

      dimension ug1i(nromx), vg1i(nromx),
     &          ug2i(nromx), vg2i(nromx),
     &          us1i(nromx), vs1i(nromx),
     &          us2i(nromx), vs2i(nromx)

      parameter (pi = 3.14159265358979323846264338327950280)

c---- romberg convergence tolerance (actual error may be much less than this)
      data romtol / 1.0e-6 /
ccc      data romtol / 1.0e-12 /

c---- lampshade meridional length**2
      delsq = (x1-x2)**2 + (r1-r2)**2

c---- reference length for convergence tolerance 
      refl = 0.5*(r1 + r2)


c---- field point is assumed to be at midpoint
      xf = 0.5*(x1 + x2)
      rf = 0.5*(r1 + r2)

c---- evaluate integrals on increasingly fine grids 
c-     (start with two intervals to avoid landing right on the midpoint)
      do 100 irom=1, nromx
        nt = 2**irom

        ug1i(irom) = 0.
        vg1i(irom) = 0.
        ug2i(irom) = 0.
        vg2i(irom) = 0.
        us1i(irom) = 0.
        vs1i(irom) = 0.
        us2i(irom) = 0.
        vs2i(irom) = 0.

c------ visit the midpoints of each of the nt intervals
        do 10 it=1, nt
          t = (float(it) - 0.5) / float(nt)
          tb = 1.0 - t

          dt = 1.0/float(nt)

          xt = x1*tb + x2*t
          rt = r1*tb + r2*t

          call ring( xt, rt, xf, rf, rcore, ugt, vgt, ust, vst )

c-------- singular parts of velocities in the limit  xt,rt -> xf,rf
          dsq = (xt-xf)**2 + (rt-rf)**2
          uga =  (rt-rf)/(4.0*pi*dsq)
     &        - 0.5*log(dsq/(64.0*rf**2)) / (8.0*pi*rf)
          vga = -(xt-xf)/(4.0*pi*dsq)
          usa = -(xt-xf)/(4.0*pi*dsq)
          vsa = -(rt-rf)/(4.0*pi*dsq)
     &        - 0.5*log(dsq/      rf**2 ) / (8.0*pi*rf)

c-------- accumulate integrals, with singular parts (at t=0.5) removed
          ug1i(irom) = ug1i(irom) + dt*(ugt*tb - uga)
          vg1i(irom) = vg1i(irom) + dt*(vgt*tb - vga)
          ug2i(irom) = ug2i(irom) + dt*(ugt*t  - uga)
          vg2i(irom) = vg2i(irom) + dt*(vgt*t  - vga)
          us1i(irom) = us1i(irom) + dt*(ust*tb - usa)
          vs1i(irom) = vs1i(irom) + dt*(vst*tb - vsa)
          us2i(irom) = us2i(irom) + dt*(ust*t  - usa)
          vs2i(irom) = vs2i(irom) + dt*(vst*t  - vsa)
 10     continue

c------ romberg sequence using all previous grid results
        do krom = irom, 2, -1
c-------- weight needed to cancel lowest-order error terms in krom level
          w = 2.0**(2*(irom-krom+1))
          wm1 = w - 1.0

c-------- put richardson extrapolation for krom level into krom-1 level
          ug1i(krom-1) = (w*ug1i(krom) - ug1i(krom-1)) / wm1
          vg1i(krom-1) = (w*vg1i(krom) - vg1i(krom-1)) / wm1
          ug2i(krom-1) = (w*ug2i(krom) - ug2i(krom-1)) / wm1
          vg2i(krom-1) = (w*vg2i(krom) - vg2i(krom-1)) / wm1
          us1i(krom-1) = (w*us1i(krom) - us1i(krom-1)) / wm1
          vs1i(krom-1) = (w*vs1i(krom) - vs1i(krom-1)) / wm1
          us2i(krom-1) = (w*us2i(krom) - us2i(krom-1)) / wm1
          vs2i(krom-1) = (w*vs2i(krom) - vs2i(krom-1)) / wm1
        enddo

        if(irom.gt.1) then
c------- compare the best-current and best-previous integrals
         errug1 = ug1i(1) - ug1i(2)
         errvg1 = vg1i(1) - vg1i(2)
         errug2 = ug2i(1) - ug2i(2)
         errvg2 = vg2i(1) - vg2i(2)
         errus1 = us1i(1) - us1i(2)
         errvs1 = vs1i(1) - vs1i(2)
         errus2 = us2i(1) - us2i(2)
         errvs2 = vs2i(1) - vs2i(2)

         err = max(
     &     abs(errug1),
     &     abs(errvg1),
     &     abs(errug2),
     &     abs(errvg2),
     &     abs(errus1),
     &     abs(errvs1),
     &     abs(errus2),
     &     abs(errvs2) )

         if(err*refl .lt. romtol) go to 101
        endif
 100  continue
      write(*,*) 'lampc: romberg convergence failed.  error =', err

 101  continue

      delsq = (x1-x2)**2 + (r1-r2)**2
      dels = sqrt(delsq)

c---- analytically-integrated singular parts which were removed
      ugai = (1.0 + log(16.0*rf/dels)) / (4.0*pi*rf)
      vgai = 0.
      usai = 0.
      vsai = (1.0 + log( 2.0*rf/dels)) / (4.0*pi*rf)

c---- return final results, with removed parts added back on
      ug1 = (ug1i(1) + ugai*0.5)*dels
      vg1 = (vg1i(1) + vgai*0.5)*dels
      ug2 = (ug2i(1) + ugai*0.5)*dels
      vg2 = (vg2i(1) + vgai*0.5)*dels
      us1 = (us1i(1) + usai*0.5)*dels
      vs1 = (vs1i(1) + vsai*0.5)*dels
      us2 = (us2i(1) + usai*0.5)*dels
      vs2 = (vs2i(1) + vsai*0.5)*dels

      return
      end ! lampc



      subroutine glamp( r1,     r2,     rf, rcore,
     &         qg1, qg1_rf, 
     &         qg2, qg2_rf, 
     &         qs1, qs1_rf, 
     &         qs2, qs2_rf )
c-----------------------------------------------------------------------
c     same as lamp, but also returns velocity gradient
c-----------------------------------------------------------------------
      dimension r1(2), r2(2), rf(2)
      dimension qg1(2), qg1_rf(2,2),
     &          qg2(2), qg2_rf(2,2),
     &          qs1(2), qs1_rf(2,2),
     &          qs2(2), qs2_rf(2,2)

      parameter (nromx=10)

      dimension qg1i(2,nromx),
     &          qg2i(2,nromx),
     &          qs1i(2,nromx),
     &          qs2i(2,nromx)
      dimension 
     &  qg1i_rf(2,2,nromx),
     &  qg2i_rf(2,2,nromx),
     &  qs1i_rf(2,2,nromx),
     &  qs2i_rf(2,2,nromx) 

      dimension rt(2)
      dimension qgt(2), qgt_rt(2,2), qgt_rf(2,2),
     &          qst(2), qst_rt(2,2), qst_rf(2,2)

      parameter (pi = 3.14159265358979323846264338327950280)

c---- romberg convergence tolerance
      data romtol / 1.0e-6 /
ccc   data romtol / 1.0e-12 /

c---- reference length for convergence tolerance 
      refl = 0.5*(r1(2) + r2(2))

c---- evaluate integrals over  0..t..1  on increasingly fine grids
      do 100 irom=1, nromx
        nt = 2**irom / 2

        do k = 1, 2
          qg1i(k,irom) = 0.
          qg2i(k,irom) = 0.
          qs1i(k,irom) = 0.
          qs2i(k,irom) = 0.
          do j = 1, 2
            qg1i_rf(k,j,irom) = 0.
            qg2i_rf(k,j,irom) = 0.
            qs1i_rf(k,j,irom) = 0.
            qs2i_rf(k,j,irom) = 0.
          enddo
        enddo

c------ visit the midpoints of each of the nt intervals
        do 10 it=1, nt
          t = (float(it) - 0.5) / float(nt)
          tb = 1.0 - t

          dt = 1.0/float(nt)

          rt(1) = r1(1)*tb + r2(1)*t
          rt(2) = r1(2)*tb + r2(2)*t

c-------- get induced velocities for vortex,source ring at xt,rt
          call dring( rt(  1),    rt(  2),    rf(  1),    rf(  2),rcore,
     &     qgt(1),qgt_rt(1,1),qgt_rt(1,2),qgt_rf(1,1),qgt_rf(1,2), 
     &     qgt(2),qgt_rt(2,1),qgt_rt(2,2),qgt_rf(2,1),qgt_rf(2,2),
     &     qst(1),qst_rt(1,1),qst_rt(1,2),qst_rf(1,1),qst_rf(1,2),
     &     qst(2),qst_rt(2,1),qst_rt(2,2),qst_rf(2,1),qst_rf(2,2) )

c-------- accumulate the separate unit-gamma, unit-sigma integrals
          do k = 1, 2
           qg1i(k,irom) = qg1i(k,irom) + dt*qgt(k)*tb
           qg2i(k,irom) = qg2i(k,irom) + dt*qgt(k)*t
           qs1i(k,irom) = qs1i(k,irom) + dt*qst(k)*tb
           qs2i(k,irom) = qs2i(k,irom) + dt*qst(k)*t
           do j = 1, 2
            qg1i_rf(k,j,irom) = qg1i_rf(k,j,irom) + dt*qgt_rf(k,j)*tb
            qg2i_rf(k,j,irom) = qg2i_rf(k,j,irom) + dt*qgt_rf(k,j)*t     
            qs1i_rf(k,j,irom) = qs1i_rf(k,j,irom) + dt*qst_rf(k,j)*tb
            qs2i_rf(k,j,irom) = qs2i_rf(k,j,irom) + dt*qst_rf(k,j)*t     
           enddo
          enddo
 10     continue

c------ romberg sequence using all previous grid results
        do krom = irom, 2, -1
c-------- weight needed to cancel lowest-order error terms in krom level
          w = 2.0 ** (2*(irom-krom+1))
          wm1 = w - 1.0

c-------- put richardson extrapolation for krom level into krom-1 level
          do k = 1, 2
            qg1i(k,krom-1) = (w*qg1i(k,krom) - qg1i(k,krom-1)) / wm1
            qg2i(k,krom-1) = (w*qg2i(k,krom) - qg2i(k,krom-1)) / wm1
            qs1i(k,krom-1) = (w*qs1i(k,krom) - qs1i(k,krom-1)) / wm1
            qs2i(k,krom-1) = (w*qs2i(k,krom) - qs2i(k,krom-1)) / wm1
            do j = 1, 2
              qg1i_rf(k,j,krom-1) = (w*qg1i_rf(k,j,krom)
     &                               - qg1i_rf(k,j,krom-1)) / wm1
              qg2i_rf(k,j,krom-1) = (w*qg2i_rf(k,j,krom)
     &                               - qg2i_rf(k,j,krom-1)) / wm1
              qs1i_rf(k,j,krom-1) = (w*qs1i_rf(k,j,krom)
     &                               - qs1i_rf(k,j,krom-1)) / wm1
              qs2i_rf(k,j,krom-1) = (w*qs2i_rf(k,j,krom)
     &                               - qs2i_rf(k,j,krom-1)) / wm1
            enddo
          enddo
        enddo

        if(irom.gt.1) then
c------- compare the best-current and best-previous integrals
         errug1 = qg1i(1,1) - qg1i(1,2)
         errvg1 = qg1i(2,1) - qg1i(2,2)
         errug2 = qg2i(1,1) - qg2i(1,2)
         errvg2 = qg2i(2,1) - qg2i(2,2)
         errus1 = qs1i(1,1) - qs1i(1,2)
         errvs1 = qs1i(2,1) - qs1i(2,2)
         errus2 = qs2i(1,1) - qs2i(1,2)
         errvs2 = qs2i(2,1) - qs2i(2,2)

         err = max(
     &     abs(errug1),
     &     abs(errvg1),
     &     abs(errug2),
     &     abs(errvg2),
     &     abs(errus1),
     &     abs(errvs1),
     &     abs(errus2),
     &     abs(errvs2) )
         
c         write(13,1200) 
c     &     abs(errug1),
c     &     abs(errvg1),
c     &     abs(errug2),
c     &     abs(errvg2),
c     &     abs(errus1),
c     &     abs(errvs1),
c     &     abs(errus2),
c     &     abs(errvs2)
c 1200    format(8e16.9)

         if(err*refl .lt. romtol) go to 101
        endif
 100  continue
      write(*,*) 'glamp: romberg convergence failed.  error =', err

 101  continue

ccc      write(*,*) irom, err

c---- return best final results
      delsq = (r1(1)-r2(1))**2 + (r1(2)-r2(2))**2
      dels  = sqrt(delsq)

      do k = 1, 2
        qg1(k) = qg1i(k,1)*dels
        qg2(k) = qg2i(k,1)*dels
        qs1(k) = qs1i(k,1)*dels
        qs2(k) = qs2i(k,1)*dels
        do j = 1, 2
          qg1_rf(k,j) = qg1i_rf(k,j,1)*dels
          qg2_rf(k,j) = qg2i_rf(k,j,1)*dels
          qs1_rf(k,j) = qs1i_rf(k,j,1)*dels
          qs2_rf(k,j) = qs2i_rf(k,j,1)*dels
        enddo
      enddo

      return
      end ! glamp


      subroutine glampc( r1, r2, rcore,
     &         qg1, qg1_rf, 
     &         qg2, qg2_rf, 
     &         qs1, qs1_rf, 
     &         qs2, qs2_rf )
c-----------------------------------------------------------------------
c     same as lampc, but also returns velocity gradient
c-----------------------------------------------------------------------
      dimension r1(2), r2(2)
      dimension qg1(2), qg1_rf(2,2),
     &          qg2(2), qg2_rf(2,2),
     &          qs1(2), qs1_rf(2,2),
     &          qs2(2), qs2_rf(2,2)

      parameter (nromx=10)

      dimension qg1i(2,nromx),
     &          qg2i(2,nromx),
     &          qs1i(2,nromx),
     &          qs2i(2,nromx)
      dimension 
     &  qg1i_rf(2,2,nromx),
     &  qg2i_rf(2,2,nromx),
     &  qs1i_rf(2,2,nromx),
     &  qs2i_rf(2,2,nromx) 

      dimension rf(2)
      dimension rt(2)
      dimension qgt(2), qgt_rt(2,2), qgt_rf(2,2),
     &          qst(2), qst_rt(2,2), qst_rf(2,2)
      dimension qga(2), qga_rt(2,2), qga_rf(2,2),
     &          qsa(2), qsa_rt(2,2), qsa_rf(2,2)
      dimension dsq_rf(2)

      dimension qgai(2), qgai_rf(2,2),
     &          qsai(2), qsai_rf(2,2)

      parameter (pi = 3.14159265358979323846264338327950280)

c---- romberg convergence tolerance
      data romtol / 1.0e-6 /
ccc   data romtol / 1.0e-12 /

c---- reference length for convergence tolerance 
      refl = 0.5*(r1(2) + r2(2))

      rf(1) = 0.5*(r1(1) + r2(1))
      rf(2) = 0.5*(r1(2) + r2(2))

c---- evaluate integrals over  0..t..1  on increasingly fine grids
      do 100 irom=1, nromx
        nt = 2**irom

        do k = 1, 2
          qg1i(k,irom) = 0.
          qg2i(k,irom) = 0.
          qs1i(k,irom) = 0.
          qs2i(k,irom) = 0.
          do j = 1, 2
            qg1i_rf(k,j,irom) = 0.
            qg2i_rf(k,j,irom) = 0.
            qs1i_rf(k,j,irom) = 0.
            qs2i_rf(k,j,irom) = 0.
          enddo
        enddo

c------ visit the midpoints of each of the nt intervals
        do 10 it=1, nt
          t = (float(it) - 0.5) / float(nt)
          tb = 1.0 - t

          dt = 1.0/float(nt)

          rt(1) = r1(1)*tb + r2(1)*t
          rt(2) = r1(2)*tb + r2(2)*t

c-------- get induced velocities for vortex,source ring at xt,rt
          call dring( rt(  1),    rt(  2),    rf(  1),    rf(  2),rcore,
     &     qgt(1),qgt_rt(1,1),qgt_rt(1,2),qgt_rf(1,1),qgt_rf(1,2), 
     &     qgt(2),qgt_rt(2,1),qgt_rt(2,2),qgt_rf(2,1),qgt_rf(2,2),
     &     qst(1),qst_rt(1,1),qst_rt(1,2),qst_rf(1,1),qst_rf(1,2),
     &     qst(2),qst_rt(2,1),qst_rt(2,2),qst_rf(2,1),qst_rf(2,2) )

c-------- singular parts of velocities in the limit  xt,rt -> xf,rf
          dsq = (rt(1)-rf(1))**2 + (rt(2)-rf(2))**2
          dsq_rf(1) = -2.0*(rt(1)-rf(1))
          dsq_rf(2) = -2.0*(rt(2)-rf(2))


          pid4 = 4.0*pi*dsq
          pir8 = 8.0*pi*rf(2)
          drsq = dsq/rf(2)**2

          dr1 = (rt(1)-rf(1))/pid4
          dr2 = (rt(2)-rf(2))/pid4

          qga(1) =  dr2 - 0.5*log(drsq/64.0) / pir8
          qga(2) = -dr1
          qsa(1) = -dr1
          qsa(2) = -dr2 - 0.5*log(drsq     ) / pir8

          qga_rf(1,1) = (-dr2 - 0.5/pir8)/dsq * dsq_rf(1)
          qga_rf(1,2) = (-dr2 - 0.5/pir8)/dsq * dsq_rf(2) - 1.0/pid4
     &             + (1.0 + 0.5*log(drsq/64.0)) / (rf(2)*pir8)

          qga_rf(2,1) =   dr1            /dsq * dsq_rf(1) + 1.0/pid4
          qga_rf(2,2) =   dr1            /dsq * dsq_rf(2)

          qsa_rf(1,1) =   dr1            /dsq * dsq_rf(1) + 1.0/pid4
          qsa_rf(1,2) =   dr1            /dsq * dsq_rf(2)

          qsa_rf(2,1) = ( dr2 - 0.5/pir8)/dsq * dsq_rf(1)
          qsa_rf(2,2) = ( dr2 - 0.5/pir8)/dsq * dsq_rf(2) + 1.0/pid4
     &             + (1.0 + 0.5*log(drsq     )) / (rf(2)*pir8)

c-------- accumulate integrals, with singular parts (at t=0.5) removed
          do k = 1, 2
           qg1i(k,irom) = qg1i(k,irom) + dt*(qgt(k)*tb - qga(k))
           qg2i(k,irom) = qg2i(k,irom) + dt*(qgt(k)*t  - qga(k))
           qs1i(k,irom) = qs1i(k,irom) + dt*(qst(k)*tb - qsa(k))
           qs2i(k,irom) = qs2i(k,irom) + dt*(qst(k)*t  - qsa(k))
           do j = 1, 2
            qg1i_rf(k,j,irom) =
     &      qg1i_rf(k,j,irom) + dt*(qgt_rf(k,j)*tb - qga_rf(k,j))
            qg2i_rf(k,j,irom) =
     &      qg2i_rf(k,j,irom) + dt*(qgt_rf(k,j)*t  - qga_rf(k,j))
            qs1i_rf(k,j,irom) =
     &      qs1i_rf(k,j,irom) + dt*(qst_rf(k,j)*tb - qsa_rf(k,j))
            qs2i_rf(k,j,irom) =
     &      qs2i_rf(k,j,irom) + dt*(qst_rf(k,j)*t  - qsa_rf(k,j))        
           enddo
          enddo
 10     continue

c------ romberg sequence using all previous grid results
        do krom = irom, 2, -1
c-------- weight needed to cancel lowest-order error terms in krom level
          w = 2.0 ** (2*(irom-krom+1))
          wm1 = w - 1.0

c-------- put richardson extrapolation for krom level into krom-1 level
          do k = 1, 2
            qg1i(k,krom-1) = (w*qg1i(k,krom) - qg1i(k,krom-1)) / wm1
            qg2i(k,krom-1) = (w*qg2i(k,krom) - qg2i(k,krom-1)) / wm1
            qs1i(k,krom-1) = (w*qs1i(k,krom) - qs1i(k,krom-1)) / wm1
            qs2i(k,krom-1) = (w*qs2i(k,krom) - qs2i(k,krom-1)) / wm1
            do j = 1, 2
              qg1i_rf(k,j,krom-1) = (w*qg1i_rf(k,j,krom)
     &                               - qg1i_rf(k,j,krom-1)) / wm1
              qg2i_rf(k,j,krom-1) = (w*qg2i_rf(k,j,krom)
     &                               - qg2i_rf(k,j,krom-1)) / wm1
              qs1i_rf(k,j,krom-1) = (w*qs1i_rf(k,j,krom)
     &                               - qs1i_rf(k,j,krom-1)) / wm1
              qs2i_rf(k,j,krom-1) = (w*qs2i_rf(k,j,krom)
     &                               - qs2i_rf(k,j,krom-1)) / wm1
            enddo
          enddo
        enddo

        if(irom.gt.1) then
c------- compare the best-current and best-previous integrals
         errug1 = qg1i(1,1) - qg1i(1,2)
         errvg1 = qg1i(2,1) - qg1i(2,2)
         errug2 = qg2i(1,1) - qg2i(1,2)
         errvg2 = qg2i(2,1) - qg2i(2,2)
         errus1 = qs1i(1,1) - qs1i(1,2)
         errvs1 = qs1i(2,1) - qs1i(2,2)
         errus2 = qs2i(1,1) - qs2i(1,2)
         errvs2 = qs2i(2,1) - qs2i(2,2)

         err = max(
     &     abs(errug1),
     &     abs(errvg1),
     &     abs(errug2),
     &     abs(errvg2),
     &     abs(errus1),
     &     abs(errvs1),
     &     abs(errus2),
     &     abs(errvs2) )
         
c         write(13,1200) 
c     &     abs(errug1),
c     &     abs(errvg1),
c     &     abs(errug2),
c     &     abs(errvg2),
c     &     abs(errus1),
c     &     abs(errvs1),
c     &     abs(errus2),
c     &     abs(errvs2)
c 1200    format(8e16.9)

         if(err*refl .lt. romtol) go to 101
        endif
 100  continue
      write(*,*) 'glampc: romberg convergence failed.  error =', err

 101  continue

ccc      write(*,*) irom, err

c---- return best final results
      delsq = (r1(1)-r2(1))**2 + (r1(2)-r2(2))**2
      dels  = sqrt(delsq)

c---- analytically-integrated singular parts which were removed
      qgai(1) = (1.0 + log(16.0*rf(2)/dels)) / (8.0*pi*rf(2))
      qgai(2) = 0.
      qsai(1) = 0.
      qsai(2) = (1.0 + log( 2.0*rf(2)/dels)) / (8.0*pi*rf(2))

      qgai_rf(1,1) = 0.
      qgai_rf(2,1) = 0.
      qsai_rf(1,1) = 0.
      qsai_rf(2,1) = 0.

      qgai_rf(1,2) = (1.0/rf(2)) / (8.0*pi*rf(2)) - qgai(1)/rf(2)
      qgai_rf(2,2) = 0.
      qsai_rf(1,2) = 0.
      qsai_rf(2,2) = (1.0/rf(2)) / (8.0*pi*rf(2)) - qsai(2)/rf(2)


      do k = 1, 2
        qg1(k) = (qg1i(k,1) + qgai(k))*dels
        qg2(k) = (qg2i(k,1) + qgai(k))*dels
        qs1(k) = (qs1i(k,1) + qsai(k))*dels
        qs2(k) = (qs2i(k,1) + qsai(k))*dels
        do j = 1, 2
          qg1_rf(k,j) = (qg1i_rf(k,j,1) + qgai_rf(k,j))*dels
          qg2_rf(k,j) = (qg2i_rf(k,j,1) + qgai_rf(k,j))*dels
          qs1_rf(k,j) = (qs1i_rf(k,j,1) + qsai_rf(k,j))*dels
          qs2_rf(k,j) = (qs2i_rf(k,j,1) + qsai_rf(k,j))*dels
        enddo
      enddo

      return
      end ! glampc



      subroutine dlamp( r1,     r2,     rf, rcore,
     &         qg1, qg1_r1, qg1_r2, qg1_rf, 
     &         qg2, qg2_r1, qg2_r2, qg2_rf, 
     &         qs1, qs1_r1, qs1_r2, qs1_rf, 
     &         qs2, qs2_r1, qs2_r2, qs2_rf )
c-----------------------------------------------------------------------
c     same as lamp, but also returns derivatives.
c-----------------------------------------------------------------------
      dimension r1(2), r2(2), rf(2)
      dimension qg1(2), qg1_r1(2,2), qg1_r2(2,2), qg1_rf(2,2),
     &          qg2(2), qg2_r1(2,2), qg2_r2(2,2), qg2_rf(2,2),
     &          qs1(2), qs1_r1(2,2), qs1_r2(2,2), qs1_rf(2,2),
     &          qs2(2), qs2_r1(2,2), qs2_r2(2,2), qs2_rf(2,2)

      parameter (nromx=10)

      dimension qg1i(2,nromx),
     &          qg2i(2,nromx),
     &          qs1i(2,nromx),
     &          qs2i(2,nromx)
      dimension 
     &  qg1i_r1(2,2,nromx),qg1i_r2(2,2,nromx),qg1i_rf(2,2,nromx),
     &  qg2i_r1(2,2,nromx),qg2i_r2(2,2,nromx),qg2i_rf(2,2,nromx),
     &  qs1i_r1(2,2,nromx),qs1i_r2(2,2,nromx),qs1i_rf(2,2,nromx),
     &  qs2i_r1(2,2,nromx),qs2i_r2(2,2,nromx),qs2i_rf(2,2,nromx) 

      dimension rt(2)
      dimension qgt(2), qgt_rt(2,2), qgt_rf(2,2),
     &          qst(2), qst_rt(2,2), qst_rf(2,2)
      dimension dels_r1(2), dels_r2(2)

      parameter (pi = 3.14159265358979323846264338327950280)

c---- romberg convergence tolerance
      data romtol / 1.0e-6 /
ccc   data romtol / 1.0e-12 /

c---- reference length for convergence tolerance 
      refl = 0.5*(r1(2) + r2(2))

c---- evaluate integrals over  0..t..1  on increasingly fine grids
      do 100 irom=1, nromx
        nt = 2**irom / 2

        do k = 1, 2
          qg1i(k,irom) = 0.
          qg2i(k,irom) = 0.
          qs1i(k,irom) = 0.
          qs2i(k,irom) = 0.
          do j = 1, 2
            qg1i_r1(k,j,irom) = 0.
            qg2i_r1(k,j,irom) = 0.
            qs1i_r1(k,j,irom) = 0.
            qs2i_r1(k,j,irom) = 0.
            qg1i_r2(k,j,irom) = 0.
            qg2i_r2(k,j,irom) = 0.
            qs1i_r2(k,j,irom) = 0.
            qs2i_r2(k,j,irom) = 0.
            qg1i_rf(k,j,irom) = 0.
            qg2i_rf(k,j,irom) = 0.
            qs1i_rf(k,j,irom) = 0.
            qs2i_rf(k,j,irom) = 0.
          enddo
        enddo

c------ visit the midpoints of each of the nt intervals
        do 10 it=1, nt
          t = (float(it) - 0.5) / float(nt)
          tb = 1.0 - t

          dt = 1.0/float(nt)

          rt(1) = r1(1)*tb + r2(1)*t
          rt(2) = r1(2)*tb + r2(2)*t

c-------- get induced velocities for vortex,source ring at xt,rt
          call dring( rt(  1),    rt(  2),    rf(  1),    rf(  2),rcore,
     &     qgt(1),qgt_rt(1,1),qgt_rt(1,2),qgt_rf(1,1),qgt_rf(1,2), 
     &     qgt(2),qgt_rt(2,1),qgt_rt(2,2),qgt_rf(2,1),qgt_rf(2,2),
     &     qst(1),qst_rt(1,1),qst_rt(1,2),qst_rf(1,1),qst_rf(1,2),
     &     qst(2),qst_rt(2,1),qst_rt(2,2),qst_rf(2,1),qst_rf(2,2) )

c-------- accumulate the separate unit-gamma, unit-sigma integrals
          do k = 1, 2
           qg1i(k,irom) = qg1i(k,irom) + dt*qgt(k)*tb
           qg2i(k,irom) = qg2i(k,irom) + dt*qgt(k)*t
           qs1i(k,irom) = qs1i(k,irom) + dt*qst(k)*tb
           qs2i(k,irom) = qs2i(k,irom) + dt*qst(k)*t
           do j = 1, 2
            qg1i_r1(k,j,irom) = qg1i_r1(k,j,irom) + dt*qgt_rt(k,j)*tb*tb
            qg2i_r1(k,j,irom) = qg2i_r1(k,j,irom) + dt*qgt_rt(k,j)*t *tb 
            qs1i_r1(k,j,irom) = qs1i_r1(k,j,irom) + dt*qst_rt(k,j)*tb*tb
            qs2i_r1(k,j,irom) = qs2i_r1(k,j,irom) + dt*qst_rt(k,j)*t *tb 
c        
            qg1i_r2(k,j,irom) = qg1i_r2(k,j,irom) + dt*qgt_rt(k,j)*tb*t
            qg2i_r2(k,j,irom) = qg2i_r2(k,j,irom) + dt*qgt_rt(k,j)*t *t  
            qs1i_r2(k,j,irom) = qs1i_r2(k,j,irom) + dt*qst_rt(k,j)*tb*t
            qs2i_r2(k,j,irom) = qs2i_r2(k,j,irom) + dt*qst_rt(k,j)*t *t  
c        
            qg1i_rf(k,j,irom) = qg1i_rf(k,j,irom) + dt*qgt_rf(k,j)*tb
            qg2i_rf(k,j,irom) = qg2i_rf(k,j,irom) + dt*qgt_rf(k,j)*t     
            qs1i_rf(k,j,irom) = qs1i_rf(k,j,irom) + dt*qst_rf(k,j)*tb
            qs2i_rf(k,j,irom) = qs2i_rf(k,j,irom) + dt*qst_rf(k,j)*t     
           enddo
          enddo
 10     continue

c------ romberg sequence using all previous grid results
        do krom = irom, 2, -1
c-------- weight needed to cancel lowest-order error terms in krom level
          w = 2.0 ** (2*(irom-krom+1))
          wm1 = w - 1.0

c-------- put richardson extrapolation for krom level into krom-1 level
          do k = 1, 2
            qg1i(k,krom-1) = (w*qg1i(k,krom) - qg1i(k,krom-1)) / wm1
            qg2i(k,krom-1) = (w*qg2i(k,krom) - qg2i(k,krom-1)) / wm1
            qs1i(k,krom-1) = (w*qs1i(k,krom) - qs1i(k,krom-1)) / wm1
            qs2i(k,krom-1) = (w*qs2i(k,krom) - qs2i(k,krom-1)) / wm1
            do j = 1, 2
              qg1i_r1(k,j,krom-1) = (w*qg1i_r1(k,j,krom)
     &                               - qg1i_r1(k,j,krom-1)) / wm1
              qg2i_r1(k,j,krom-1) = (w*qg2i_r1(k,j,krom)
     &                               - qg2i_r1(k,j,krom-1)) / wm1
              qs1i_r1(k,j,krom-1) = (w*qs1i_r1(k,j,krom)
     &                               - qs1i_r1(k,j,krom-1)) / wm1
              qs2i_r1(k,j,krom-1) = (w*qs2i_r1(k,j,krom)
     &                               - qs2i_r1(k,j,krom-1)) / wm1

              qg1i_r2(k,j,krom-1) = (w*qg1i_r2(k,j,krom)
     &                               - qg1i_r2(k,j,krom-1)) / wm1
              qg2i_r2(k,j,krom-1) = (w*qg2i_r2(k,j,krom)
     &                               - qg2i_r2(k,j,krom-1)) / wm1
              qs1i_r2(k,j,krom-1) = (w*qs1i_r2(k,j,krom)
     &                               - qs1i_r2(k,j,krom-1)) / wm1
              qs2i_r2(k,j,krom-1) = (w*qs2i_r2(k,j,krom)
     &                               - qs2i_r2(k,j,krom-1)) / wm1

              qg1i_rf(k,j,krom-1) = (w*qg1i_rf(k,j,krom)
     &                               - qg1i_rf(k,j,krom-1)) / wm1
              qg2i_rf(k,j,krom-1) = (w*qg2i_rf(k,j,krom)
     &                               - qg2i_rf(k,j,krom-1)) / wm1
              qs1i_rf(k,j,krom-1) = (w*qs1i_rf(k,j,krom)
     &                               - qs1i_rf(k,j,krom-1)) / wm1
              qs2i_rf(k,j,krom-1) = (w*qs2i_rf(k,j,krom)
     &                               - qs2i_rf(k,j,krom-1)) / wm1
            enddo
          enddo
        enddo

        if(irom.gt.1) then
c------- compare the best-current and best-previous integrals
         errug1 = qg1i(1,1) - qg1i(1,2)
         errvg1 = qg1i(2,1) - qg1i(2,2)
         errug2 = qg2i(1,1) - qg2i(1,2)
         errvg2 = qg2i(2,1) - qg2i(2,2)
         errus1 = qs1i(1,1) - qs1i(1,2)
         errvs1 = qs1i(2,1) - qs1i(2,2)
         errus2 = qs2i(1,1) - qs2i(1,2)
         errvs2 = qs2i(2,1) - qs2i(2,2)

         err = max(
     &     abs(errug1),
     &     abs(errvg1),
     &     abs(errug2),
     &     abs(errvg2),
     &     abs(errus1),
     &     abs(errvs1),
     &     abs(errus2),
     &     abs(errvs2) )
         
c         write(13,1200) 
c     &     abs(errug1),
c     &     abs(errvg1),
c     &     abs(errug2),
c     &     abs(errvg2),
c     &     abs(errus1),
c     &     abs(errvs1),
c     &     abs(errus2),
c     &     abs(errvs2)
c 1200    format(8e16.9)

         if(err*refl .lt. romtol) go to 101
        endif
 100  continue
      write(*,*) 'dlamp: romberg convergence failed.  error =', err

 101  continue

ccc      write(*,*) irom, err

c---- return best final results
      delsq = (r1(1)-r2(1))**2 + (r1(2)-r2(2))**2
      dels  = sqrt(delsq)
      dels_r1(1) =  (r1(1)-r2(1))/dels
      dels_r2(1) = -(r1(1)-r2(1))/dels
      dels_r1(2) =  (r1(2)-r2(2))/dels
      dels_r2(2) = -(r1(2)-r2(2))/dels

      do k = 1, 2
        qg1(k) = qg1i(k,1)*dels
        qg2(k) = qg2i(k,1)*dels
        qs1(k) = qs1i(k,1)*dels
        qs2(k) = qs2i(k,1)*dels
        do j = 1, 2
          qg1_r1(k,j) = qg1i_r1(k,j,1)*dels + qg1i(k,1)*dels_r1(j)
          qg2_r1(k,j) = qg2i_r1(k,j,1)*dels + qg2i(k,1)*dels_r1(j)
          qs1_r1(k,j) = qs1i_r1(k,j,1)*dels + qs1i(k,1)*dels_r1(j)
          qs2_r1(k,j) = qs2i_r1(k,j,1)*dels + qs2i(k,1)*dels_r1(j)

          qg1_r2(k,j) = qg1i_r2(k,j,1)*dels + qg1i(k,1)*dels_r2(j)
          qg2_r2(k,j) = qg2i_r2(k,j,1)*dels + qg2i(k,1)*dels_r2(j)
          qs1_r2(k,j) = qs1i_r2(k,j,1)*dels + qs1i(k,1)*dels_r2(j)
          qs2_r2(k,j) = qs2i_r2(k,j,1)*dels + qs2i(k,1)*dels_r2(j)

          qg1_rf(k,j) = qg1i_rf(k,j,1)*dels
          qg2_rf(k,j) = qg2i_rf(k,j,1)*dels
          qs1_rf(k,j) = qs1i_rf(k,j,1)*dels
          qs2_rf(k,j) = qs2i_rf(k,j,1)*dels
        enddo
      enddo

      return
      end ! dlamp



      subroutine dlampc( r1,     r2, rcore,
     &          qg1, qg1_r1, qg1_r2,
     &          qg2, qg2_r1, qg2_r2,
     &          qs1, qs1_r1, qs1_r2,
     &          qs2, qs2_r1, qs2_r2 )
c-----------------------------------------------------------------------
c     same as lampc, but also returns derivatives.
c-----------------------------------------------------------------------
      dimension r1(2), r2(2)
      dimension qg1(2), qg1_r1(2,2), qg1_r2(2,2),
     &          qg2(2), qg2_r1(2,2), qg2_r2(2,2),
     &          qs1(2), qs1_r1(2,2), qs1_r2(2,2),
     &          qs2(2), qs2_r1(2,2), qs2_r2(2,2)

      parameter (nromx=10)

      dimension qg1i(2,nromx),
     &          qg2i(2,nromx),
     &          qs1i(2,nromx),
     &          qs2i(2,nromx)
      dimension 
     &  qg1i_r1(2,2,nromx),qg1i_r2(2,2,nromx),qg1i_rf(2,2,nromx),
     &  qg2i_r1(2,2,nromx),qg2i_r2(2,2,nromx),qg2i_rf(2,2,nromx),
     &  qs1i_r1(2,2,nromx),qs1i_r2(2,2,nromx),qs1i_rf(2,2,nromx),
     &  qs2i_r1(2,2,nromx),qs2i_r2(2,2,nromx),qs2i_rf(2,2,nromx) 

      dimension rf(2)
      dimension rt(2)
      dimension qgt(2), qgt_rt(2,2), qgt_rf(2,2),
     &          qst(2), qst_rt(2,2), qst_rf(2,2)
      dimension qga(2), qga_rt(2,2), qga_rf(2,2),
     &          qsa(2), qsa_rt(2,2), qsa_rf(2,2)
      dimension dsq_rt(2), dsq_rf(2), dels_r1(2), dels_r2(2)

      dimension qgai(2), qgai_r1(2,2), qgai_r2(2,2), qgai_rf(2,2),
     &          qsai(2), qsai_r1(2,2), qsai_r2(2,2), qsai_rf(2,2)

      parameter (pi = 3.14159265358979323846264338327950280)

c---- romberg convergence tolerance
      data romtol / 1.0e-6 /
ccc   data romtol / 1.0e-12 /

c---- reference length for convergence tolerance 
      refl = 0.5*(r1(2) + r2(2))

      rf(1) = 0.5*(r1(1) + r2(1))
      rf(2) = 0.5*(r1(2) + r2(2))

c---- evaluate integrals over  0..t..1  on increasingly fine grids
      do 100 irom=1, nromx
        nt = 2**irom

        do k = 1, 2
          qg1i(k,irom) = 0.
          qg2i(k,irom) = 0.
          qs1i(k,irom) = 0.
          qs2i(k,irom) = 0.
          do j = 1, 2
            qg1i_r1(k,j,irom) = 0.
            qg2i_r1(k,j,irom) = 0.
            qs1i_r1(k,j,irom) = 0.
            qs2i_r1(k,j,irom) = 0.
            qg1i_r2(k,j,irom) = 0.
            qg2i_r2(k,j,irom) = 0.
            qs1i_r2(k,j,irom) = 0.
            qs2i_r2(k,j,irom) = 0.
            qg1i_rf(k,j,irom) = 0.
            qg2i_rf(k,j,irom) = 0.
            qs1i_rf(k,j,irom) = 0.
            qs2i_rf(k,j,irom) = 0.
          enddo
        enddo

c------ visit the midpoints of each of the nt intervals
        do 10 it=1, nt
          t = (float(it) - 0.5) / float(nt)
          tb = 1.0 - t

          dt = 1.0/float(nt)

          rt(1) = r1(1)*tb + r2(1)*t
          rt(2) = r1(2)*tb + r2(2)*t

c-------- get induced velocities for vortex,source ring at xt,rt
          call dring( rt(  1),    rt(  2),    rf(  1),    rf(  2),rcore,
     &     qgt(1),qgt_rt(1,1),qgt_rt(1,2),qgt_rf(1,1),qgt_rf(1,2), 
     &     qgt(2),qgt_rt(2,1),qgt_rt(2,2),qgt_rf(2,1),qgt_rf(2,2),
     &     qst(1),qst_rt(1,1),qst_rt(1,2),qst_rf(1,1),qst_rf(1,2),
     &     qst(2),qst_rt(2,1),qst_rt(2,2),qst_rf(2,1),qst_rf(2,2) )

c-------- singular parts of velocities in the limit  xt,rt -> xf,rf
          dsq = (rt(1)-rf(1))**2 + (rt(2)-rf(2))**2
          dsq_rt(1) =  2.0*(rt(1)-rf(1))
          dsq_rt(2) =  2.0*(rt(2)-rf(2))
          dsq_rf(1) = -2.0*(rt(1)-rf(1))
          dsq_rf(2) = -2.0*(rt(2)-rf(2))


          pid4 = 4.0*pi*dsq
          pir8 = 8.0*pi*rf(2)
          drsq = dsq/rf(2)**2

          dr1 = (rt(1)-rf(1))/pid4
          dr2 = (rt(2)-rf(2))/pid4

          qga(1) =  dr2 - 0.5*log(drsq/64.0) / pir8
          qga(2) = -dr1
          qsa(1) = -dr1
          qsa(2) = -dr2 - 0.5*log(drsq     ) / pir8

          qga_rt(1,1) = (-dr2 - 0.5/pir8)/dsq * dsq_rt(1)
          qga_rt(1,2) = (-dr2 - 0.5/pir8)/dsq * dsq_rt(2) + 1.0/pid4
          qga_rf(1,1) = (-dr2 - 0.5/pir8)/dsq * dsq_rf(1)
          qga_rf(1,2) = (-dr2 - 0.5/pir8)/dsq * dsq_rf(2) - 1.0/pid4
     &             + (1.0 + 0.5*log(drsq/64.0)) / (rf(2)*pir8)

          qga_rt(2,1) =   dr1            /dsq * dsq_rt(1) - 1.0/pid4
          qga_rt(2,2) =   dr1            /dsq * dsq_rt(2)
          qga_rf(2,1) =   dr1            /dsq * dsq_rf(1) + 1.0/pid4
          qga_rf(2,2) =   dr1            /dsq * dsq_rf(2)

          qsa_rt(1,1) =   dr1            /dsq * dsq_rt(1) - 1.0/pid4
          qsa_rt(1,2) =   dr1            /dsq * dsq_rt(2)
          qsa_rf(1,1) =   dr1            /dsq * dsq_rf(1) + 1.0/pid4
          qsa_rf(1,2) =   dr1            /dsq * dsq_rf(2)

          qsa_rt(2,1) = ( dr2 - 0.5/pir8)/dsq * dsq_rt(1)
          qsa_rt(2,2) = ( dr2 - 0.5/pir8)/dsq * dsq_rt(2) - 1.0/pid4
          qsa_rf(2,1) = ( dr2 - 0.5/pir8)/dsq * dsq_rf(1)
          qsa_rf(2,2) = ( dr2 - 0.5/pir8)/dsq * dsq_rf(2) + 1.0/pid4
     &             + (1.0 + 0.5*log(drsq     )) / (rf(2)*pir8)

c-------- accumulate integrals, with singular parts (at t=0.5) removed
          do k = 1, 2
           qg1i(k,irom) = qg1i(k,irom) + dt*(qgt(k)*tb - qga(k))
           qg2i(k,irom) = qg2i(k,irom) + dt*(qgt(k)*t  - qga(k))
           qs1i(k,irom) = qs1i(k,irom) + dt*(qst(k)*tb - qsa(k))
           qs2i(k,irom) = qs2i(k,irom) + dt*(qst(k)*t  - qsa(k))
           do j = 1, 2
            qg1i_r1(k,j,irom) =
     &      qg1i_r1(k,j,irom) + dt*(qgt_rt(k,j)*tb - qga_rt(k,j))*tb
            qg2i_r1(k,j,irom) =
     &      qg2i_r1(k,j,irom) + dt*(qgt_rt(k,j)*t  - qga_rt(k,j))*tb 
            qs1i_r1(k,j,irom) =
     &      qs1i_r1(k,j,irom) + dt*(qst_rt(k,j)*tb - qsa_rt(k,j))*tb
            qs2i_r1(k,j,irom) =
     &      qs2i_r1(k,j,irom) + dt*(qst_rt(k,j)*t  - qsa_rt(k,j))*tb 
c        
            qg1i_r2(k,j,irom) =
     &      qg1i_r2(k,j,irom) + dt*(qgt_rt(k,j)*tb - qga_rt(k,j))*t
            qg2i_r2(k,j,irom) =
     &      qg2i_r2(k,j,irom) + dt*(qgt_rt(k,j)*t  - qga_rt(k,j))*t
            qs1i_r2(k,j,irom) =
     &      qs1i_r2(k,j,irom) + dt*(qst_rt(k,j)*tb - qsa_rt(k,j))*t
            qs2i_r2(k,j,irom) =
     &      qs2i_r2(k,j,irom) + dt*(qst_rt(k,j)*t  - qsa_rt(k,j))*t
c        
            qg1i_rf(k,j,irom) =
     &      qg1i_rf(k,j,irom) + dt*(qgt_rf(k,j)*tb - qga_rf(k,j))
            qg2i_rf(k,j,irom) =
     &      qg2i_rf(k,j,irom) + dt*(qgt_rf(k,j)*t  - qga_rf(k,j))
            qs1i_rf(k,j,irom) =
     &      qs1i_rf(k,j,irom) + dt*(qst_rf(k,j)*tb - qsa_rf(k,j))
            qs2i_rf(k,j,irom) =
     &      qs2i_rf(k,j,irom) + dt*(qst_rf(k,j)*t  - qsa_rf(k,j))        
           enddo
          enddo
 10     continue

c------ romberg sequence using all previous grid results
        do krom = irom, 2, -1
c-------- weight needed to cancel lowest-order error terms in krom level
          w = 2.0 ** (2*(irom-krom+1))
          wm1 = w - 1.0

c-------- put richardson extrapolation for krom level into krom-1 level
          do k = 1, 2
            qg1i(k,krom-1) = (w*qg1i(k,krom) - qg1i(k,krom-1)) / wm1
            qg2i(k,krom-1) = (w*qg2i(k,krom) - qg2i(k,krom-1)) / wm1
            qs1i(k,krom-1) = (w*qs1i(k,krom) - qs1i(k,krom-1)) / wm1
            qs2i(k,krom-1) = (w*qs2i(k,krom) - qs2i(k,krom-1)) / wm1
            do j = 1, 2
              qg1i_r1(k,j,krom-1) = (w*qg1i_r1(k,j,krom)
     &                               - qg1i_r1(k,j,krom-1)) / wm1
              qg2i_r1(k,j,krom-1) = (w*qg2i_r1(k,j,krom)
     &                               - qg2i_r1(k,j,krom-1)) / wm1
              qs1i_r1(k,j,krom-1) = (w*qs1i_r1(k,j,krom)
     &                               - qs1i_r1(k,j,krom-1)) / wm1
              qs2i_r1(k,j,krom-1) = (w*qs2i_r1(k,j,krom)
     &                               - qs2i_r1(k,j,krom-1)) / wm1

              qg1i_r2(k,j,krom-1) = (w*qg1i_r2(k,j,krom)
     &                               - qg1i_r2(k,j,krom-1)) / wm1
              qg2i_r2(k,j,krom-1) = (w*qg2i_r2(k,j,krom)
     &                               - qg2i_r2(k,j,krom-1)) / wm1
              qs1i_r2(k,j,krom-1) = (w*qs1i_r2(k,j,krom)
     &                               - qs1i_r2(k,j,krom-1)) / wm1
              qs2i_r2(k,j,krom-1) = (w*qs2i_r2(k,j,krom)
     &                               - qs2i_r2(k,j,krom-1)) / wm1

              qg1i_rf(k,j,krom-1) = (w*qg1i_rf(k,j,krom)
     &                               - qg1i_rf(k,j,krom-1)) / wm1
              qg2i_rf(k,j,krom-1) = (w*qg2i_rf(k,j,krom)
     &                               - qg2i_rf(k,j,krom-1)) / wm1
              qs1i_rf(k,j,krom-1) = (w*qs1i_rf(k,j,krom)
     &                               - qs1i_rf(k,j,krom-1)) / wm1
              qs2i_rf(k,j,krom-1) = (w*qs2i_rf(k,j,krom)
     &                               - qs2i_rf(k,j,krom-1)) / wm1
            enddo
          enddo
        enddo

        if(irom.gt.1) then
c------- compare the best-current and best-previous integrals
         errug1 = qg1i(1,1) - qg1i(1,2)
         errvg1 = qg1i(2,1) - qg1i(2,2)
         errug2 = qg2i(1,1) - qg2i(1,2)
         errvg2 = qg2i(2,1) - qg2i(2,2)
         errus1 = qs1i(1,1) - qs1i(1,2)
         errvs1 = qs1i(2,1) - qs1i(2,2)
         errus2 = qs2i(1,1) - qs2i(1,2)
         errvs2 = qs2i(2,1) - qs2i(2,2)

         err = max(
     &     abs(errug1),
     &     abs(errvg1),
     &     abs(errug2),
     &     abs(errvg2),
     &     abs(errus1),
     &     abs(errvs1),
     &     abs(errus2),
     &     abs(errvs2) )
         
c         write(13,1200) 
c     &     abs(errug1),
c     &     abs(errvg1),
c     &     abs(errug2),
c     &     abs(errvg2),
c     &     abs(errus1),
c     &     abs(errvs1),
c     &     abs(errus2),
c     &     abs(errvs2)
c 1200    format(8e16.9)

         if(err*refl .lt. romtol) go to 101
        endif
 100  continue
      write(*,*) 'dlampc: romberg convergence failed.  error =', err

 101  continue

ccc      write(*,*) irom, err

c---- return best final results
      delsq = (r1(1)-r2(1))**2 + (r1(2)-r2(2))**2
      dels  = sqrt(delsq)
      dels_r1(1) =  (r1(1)-r2(1))/dels
      dels_r2(1) = -(r1(1)-r2(1))/dels
      dels_r1(2) =  (r1(2)-r2(2))/dels
      dels_r2(2) = -(r1(2)-r2(2))/dels

c---- analytically-integrated singular parts which were removed
      qgai(1) = (1.0 + log(16.0*rf(2)/dels)) / (8.0*pi*rf(2))
      qgai(2) = 0.
      qsai(1) = 0.
      qsai(2) = (1.0 + log( 2.0*rf(2)/dels)) / (8.0*pi*rf(2))
      do j = 1, 2
        qgai_r1(1,j) = (-dels_r1(j)/dels) / (8.0*pi*rf(2))
        qgai_r1(2,j) = 0.
        qsai_r1(1,j) = 0.
        qsai_r1(2,j) = (-dels_r1(j)/dels) / (8.0*pi*rf(2))

        qgai_r2(1,j) = (-dels_r2(j)/dels) / (8.0*pi*rf(2))
        qgai_r2(2,j) = 0.
        qsai_r2(1,j) = 0.
        qsai_r2(2,j) = (-dels_r2(j)/dels) / (8.0*pi*rf(2))
      enddo

      qgai_rf(1,1) = 0.
      qgai_rf(2,1) = 0.
      qsai_rf(1,1) = 0.
      qsai_rf(2,1) = 0.

      qgai_rf(1,2) = (1.0/rf(2)) / (8.0*pi*rf(2)) - qgai(1)/rf(2)
      qgai_rf(2,2) = 0.
      qsai_rf(1,2) = 0.
      qsai_rf(2,2) = (1.0/rf(2)) / (8.0*pi*rf(2)) - qsai(2)/rf(2)


      do k = 1, 2
        qg1(k) = (qg1i(k,1) + qgai(k))*dels
        qg2(k) = (qg2i(k,1) + qgai(k))*dels
        qs1(k) = (qs1i(k,1) + qsai(k))*dels
        qs2(k) = (qs2i(k,1) + qsai(k))*dels
        do j = 1, 2
          qg1_r1(k,j) = (qg1i_r1(k,j,1) + qgai_r1(k,j))*dels
     &                + (qg1i   (k,  1) + qgai   (k  ))*dels_r1(j)
     &            + 0.5*(qg1i_rf(k,j,1) + qgai_rf(k,j))*dels
          qg2_r1(k,j) = (qg2i_r1(k,j,1) + qgai_r1(k,j))*dels
     &                + (qg2i   (k,  1) + qgai   (k  ))*dels_r1(j)
     &            + 0.5*(qg2i_rf(k,j,1) + qgai_rf(k,j))*dels
          qs1_r1(k,j) = (qs1i_r1(k,j,1) + qsai_r1(k,j))*dels
     &                + (qs1i   (k,  1) + qsai   (k  ))*dels_r1(j)
     &            + 0.5*(qs1i_rf(k,j,1) + qsai_rf(k,j))*dels
          qs2_r1(k,j) = (qs2i_r1(k,j,1) + qsai_r1(k,j))*dels
     &                + (qs2i   (k,  1) + qsai   (k  ))*dels_r1(j)
     &            + 0.5*(qs2i_rf(k,j,1) + qsai_rf(k,j))*dels

          qg1_r2(k,j) = (qg1i_r2(k,j,1) + qgai_r2(k,j))*dels
     &                + (qg1i   (k,  1) + qgai   (k  ))*dels_r2(j)
     &            + 0.5*(qg1i_rf(k,j,1) + qgai_rf(k,j))*dels
          qg2_r2(k,j) = (qg2i_r2(k,j,1) + qgai_r2(k,j))*dels
     &                + (qg2i   (k,  1) + qgai   (k  ))*dels_r2(j)
     &            + 0.5*(qg2i_rf(k,j,1) + qgai_rf(k,j))*dels
          qs1_r2(k,j) = (qs1i_r2(k,j,1) + qsai_r2(k,j))*dels
     &                + (qs1i   (k,  1) + qsai   (k  ))*dels_r2(j)
     &            + 0.5*(qs1i_rf(k,j,1) + qsai_rf(k,j))*dels
          qs2_r2(k,j) = (qs2i_r2(k,j,1) + qsai_r2(k,j))*dels
     &                + (qs2i   (k,  1) + qsai   (k  ))*dels_r2(j)
     &            + 0.5*(qs2i_rf(k,j,1) + qsai_rf(k,j))*dels
        enddo
      enddo

      return
      end ! dlampc




      subroutine ring( xv, rv, xf, rf, rcore, ux, ur, sx, sr )
c-----------------------------------------------------------------------
c     Computes the velocities induced by a vortex/source ring 
c     located at xv with radius rv with unit circulation 
c     and unit source/length density.
c
c     Adapted from routines provided by J. Kerwin.
c-----------------------------------------------------------------------
c  input:
c     xv,rv  x,r of the ring
c     xf,rf  x,r of the field point
c     rcore  core radius
c
c  Output:
c     ux,ur  velocity at xf,rf for unit circulation (+ counterclockwise)
c     sx,sr  velocity at xf,rf for unit source/perimeter
c-----------------------------------------------------------------------
      parameter (pi = 3.14159265358979323846264338327950280)

      if(rv.le.0.0) then
c------ zero-radius ring
        ux = 0.
        ur = 0.
        sx = 0.
        sr = 0.
        return
      endif

c---- this fails if r=1 and x=0  (on the ring itself)
      r = rf/rv
      x = (xv-xf)/rv
      rc = rcore/rv

      if(r .eq. 1.0 .and. x .eq. 0.0) then
       ux = 0.
       ur = 0.
       sx = 0.
       sr = 0.
       return
      endif

      if (rf .eq. 0.0) then
c----- control point on the axis
       rsq  = 1.0 + x**2
c       rsq = 1.0 + x**2 + rc**2
c       rsq = sqrt((1.0 + x**2)**2 + rc**4)

       ux = 1.0 / sqrt(rsq)**3 / (2.0*rv)
       ur = 0.0

       sx =  -x / sqrt(rsq)**3 / (2.0*rv)
       sr = 0.0

      else
c----- control point not on x-axis
c       xrp = x**2 + (1.0+r)**2
c      xrm = x**2 + (1.0-r)**2
c      xrp = x**2 + (1.0+r)**2 + rc**2
c      xrm = x**2 + (1.0-r)**2 + rc**2
       xrp = sqrt((x**2 + (1.0+r)**2)**2 + rc**4)
       xrm = sqrt((x**2 + (1.0-r)**2)**2 + rc**4)

       xrp = x**2 + (1.0+r)**2

       srp = sqrt(xrp)

       ak = xrm/xrp
       call ellek(ak,ele,elk)

       f = 2.0/xrm

       ux = ( 1.0/ srp   )*(elk - ele*(1.0 + f*(r-1.0)))/(2.0*pi*rv)
       ur = (   x/(srp*r))*(elk - ele*(1.0 + f* r     ))/(2.0*pi*rv)

       sx =  (  x/ srp   )*(    - ele*       f         )/(2.0*pi*rv)
       sr =  (1.0/(srp*r))*(elk - ele*(1.0 + f*(r-r*r)))/(2.0*pi*rv)

ccc----- streamfunction due to vortex
cc       psi = ((1.0 - 2.0*xrp*r)*elk - ele)*rv / (2.0*pi*sqrt(xrp))
      endif

      return
      end ! ring



      subroutine dring(        xv,    rv,    xf,    rf, rcore,
     &                  ux, ux_xv, ux_rv, ux_xf, ux_rf,
     &                  ur, ur_xv, ur_rv, ur_xf, ur_rf,
     &                  sx, sx_xv, sx_rv, sx_xf, sx_rf,
     &                  sr, sr_xv, sr_rv, sr_xf, sr_rf )
c-----------------------------------------------------------------------
c     Same as ring, but also returns aic derivatives w.r.t. geometry
c-----------------------------------------------------------------------
      parameter (pi = 3.14159265358979323846264338327950280)

      ux = 0.
      ur = 0.
      sx = 0.
      sr = 0.

      ux_x = 0.
      ur_x = 0.
      sx_x = 0.
      sr_x = 0.

      ux_r = 0.
      ur_r = 0.
      sx_r = 0.
      sr_r = 0.

      rvi = 0.

      if(rv.le.0.0) then
c----- zero-radius ring
       go to 90
      endif

      rvi = 1.0/rv

c---- this fails if r=1 and x=0  (on the ring itself)
      r =     rf *rvi
      x = (xv-xf)*rvi
      rc =  rcore*rvi

      if(r.eq.1.0 .and. x.eq.0.0) then
       go to 90
      endif

      if (rf .eq. 0.0) then
c----- control point on the axis
       rsq  = 1.0 + x**2
c       rsq = 1.0 + x**2 + rc**2
c       rsq = sqrt((1.0 + x**2)**2 + rc**4)

       ux = 1.0 / sqrt(rsq)**3 * pi  ! / (2.0*pi*rv)
       sx =  -x / sqrt(rsq)**3 * pi  ! / (2.0*pi*rv)
       ux_x = -3.0*x*ux/rsq
       sx_x = -3.0*x*ux/rsq - ux

      else
c----- control point not on x-axis
c      xrp = x**2 + (1.0+r)**2
c      xrm = x**2 + (1.0-r)**2
c      xrp = x**2 + (1.0+r)**2 + rc**2
c      xrm = x**2 + (1.0-r)**2 + rc**2
       xrp = sqrt((x**2 + (1.0+r)**2)**2 + rc**4)
       xrm = sqrt((x**2 + (1.0-r)**2)**2 + rc**4)

       xrp_x =  2.0*x
       xrm_x =  2.0*x
       xrp_r =  2.0*(1.0+r)
       xrm_r = -2.0*(1.0-r)

       srp = sqrt(xrp)
       srp_x = (0.5/srp)*xrp_x
       srp_r = (0.5/srp)*xrp_r

       ak   =  xrm/xrp
       ak_x = (xrm_x - ak*xrp_x)/xrp
       ak_r = (xrm_r - ak*xrp_r)/xrp

       call dellek(ak, ele, dele,
     &                 elk, delk )
       ele_x = dele*ak_x
       ele_r = dele*ak_r
       elk_x = delk*ak_x
       elk_r = delk*ak_r

       f = 2.0/xrm
       f_x = (-f/xrm)*xrm_x
       f_r = (-f/xrm)*xrm_r

       ux   = ( 1.0/ srp   )*(elk   - ele  *(1.0 + f  *(r-1.0)))
       ux_x = ( 1.0/ srp   )*(elk_x - ele_x*(1.0 + f  *(r-1.0))
     &                              - ele  *(      f_x*(r-1.0)))
     &      -   (ux/ srp)*srp_x
       ux_r = ( 1.0/ srp   )*(elk_r - ele_r*(1.0 + f  *(r-1.0))
     &                              - ele  *(      f_r*(r-1.0))
     &                              - ele  *       f           )
     &      -   (ux/srp)*srp_r

       ur   = (   x/(srp*r))*(elk   - ele  *(1.0 + f  * r     ))
       ur_x = (   x/(srp*r))*(elk_x - ele_x*(1.0 + f  * r     )
     &                              - ele  *(      f_x* r     ))
     &      + ( 1.0/(srp*r))*(elk   - ele  *(1.0 + f  * r     ))
     &      -   (ur/srp)*srp_x
       ur_r = (   x/(srp*r))*(elk_r - ele_r*(1.0 + f  * r     )
     &                              - ele  *(      f_r* r     
     &                                           + f          ))
     &      -   (ur/srp)*srp_r
     &      -    ur/r

       sx   = (   x/ srp   )*(      - ele  *       f           )
       sx_x = (   x/ srp   )*(      - ele_x*       f
     &                              - ele  *       f_x         )
     &      + ( 1.0/ srp   )*(      - ele  *       f           )
     &      -   (sx/srp)*srp_x
       sx_r = (   x/ srp   )*(      - ele_r*       f 
     &                              - ele  *       f_r         )
     &      -   (sx/srp)*srp_r

       sr   = ( 1.0/(srp*r))*(elk   - ele  *(1.0 + f  *(r-r*r)))
       sr_x = ( 1.0/(srp*r))*(elk_x - ele_x*(1.0 + f  *(r-r*r))
     &                              - ele  *(      f_x*(r-r*r)))
     &      -   (sr/ srp)*srp_x
       sr_r = ( 1.0/(srp*r))*(elk_r - ele_r*(1.0 + f  *(r-r*r))
     &                              - ele  *(      f_r*(r-r*r)
     &                                           + f  *(1.0-2.0*r)))
     &      -   (sr/srp)*srp_r
     &      -    sr/r

      endif

 90   continue
cc    x = (xv-xf)/rv
      x_xv =  rvi
      x_rv = -x*rvi
      x_xf = -rvi

cc    r = rf/rv
      r_rv = -r*rvi
      r_rf =  rvi

      hrip = rvi/(2.0*pi)

      ux    = hrip*ux
      ur    = hrip*ur
      sx    = hrip*sx
      sr    = hrip*sr

      ux_xv = hrip* ux_x*x_xv
      ur_xv = hrip* ur_x*x_xv
      sx_xv = hrip* sx_x*x_xv
      sr_xv = hrip* sr_x*x_xv
      ux_rv = hrip*(ux_x*x_rv + ux_r*r_rv) - ux*rvi
      ur_rv = hrip*(ur_x*x_rv + ur_r*r_rv) - ur*rvi
      sx_rv = hrip*(sx_x*x_rv + sx_r*r_rv) - sx*rvi
      sr_rv = hrip*(sr_x*x_rv + sr_r*r_rv) - sr*rvi
      ux_xf = hrip* ux_x*x_xf
      ur_xf = hrip* ur_x*x_xf
      sx_xf = hrip* sx_x*x_xf
      sr_xf = hrip* sr_x*x_xf
      ux_rf = hrip*             ux_r*r_rf
      ur_rf = hrip*             ur_r*r_rf
      sx_rf = hrip*             sx_r*r_rf
      sr_rf = hrip*             sr_r*r_rf

      return
      end ! dring


      subroutine ellek(ak,ele,elk)
c-----------------------------------------------------------------------
c     Elliptic functions routine

c     Adapted from routines provided by J. Kerwin.
c-----------------------------------------------------------------------
c   Input
c     ak     elliptic-integral argument

c   Output
c     ele    complete elliptic integral of the second kind
c     elk    complete elliptic integral of the first  kind
c_______________________________________________________________________

      alk = -log(ak)

      ele = 1.00000000000
     &    +(0.44325141463
     &    +(0.06260601220
     &    +(0.04757383546
     &    + 0.01736506451*ak)*ak)*ak)*ak
     &  +( (0.24998368310
     &    +(0.09200180037
     &    +(0.04069697526
     &    + 0.00526449639*ak)*ak)*ak)*ak )*alk

      elk = 1.38629436112
     &    +(0.09666344259
     &    +(0.03590092383
     &    +(0.03742563713
     &    + 0.01451196212*ak)*ak)*ak)*ak
     &  +(  0.50000000000
     &    +(0.12498593597
     &    +(0.06880248576
     &    +(0.03328355346
     &    + 0.00441787012*ak)*ak)*ak)*ak )*alk

      return
      end ! ellek



      subroutine dellek(ak, ele, ele_ak,
     &                      elk, elk_ak )
c-----------------------------------------------------------------------
c     Elliptic functions + derivative routine

c     Adapted from routines provided by J. Kerwin.
c-----------------------------------------------------------------------
c   Input
c     ak     elliptic-integral argument

c   Output
c     ele     complete elliptic integral of the second kind
c     elk     complete elliptic integral of the first  kind
c     ele_ak  d(ele)/d(ak)
c     elk_ak  d(elk)/d(ak)
c_______________________________________________________________________

      alk = -log(ak)

      ele = 1.00000000000
     &    +(0.44325141463
     &    +(0.06260601220
     &    +(0.04757383546
     &    + 0.01736506451*ak)*ak)*ak)*ak
     &  +( (0.24998368310
     &    +(0.09200180037
     &    +(0.04069697526
     &    + 0.00526449639*ak)*ak)*ak)*ak )*alk

      elk = 1.38629436112
     &    +(0.09666344259
     &    +(0.03590092383
     &    +(0.03742563713
     &    + 0.01451196212*ak)*ak)*ak)*ak
     &  +(  0.50000000000
     &    +(0.12498593597
     &    +(0.06880248576
     &    +(0.03328355346
     &    + 0.00441787012*ak)*ak)*ak)*ak )*alk

      ele_ak =
     &      0.19326773153
     &    +(0.03321022403
     &    +(0.10202453112
     &    + 0.06419576165*ak)*ak)*ak
     &  +(  0.24998368310
     &    +(0.18400360074
     &    +(0.12209092578
     &    + 0.02105798556*ak)*ak)*ak )*alk

      elk_ak =
     &     -0.028322493380
     &    +(0.002999361900
     &    +(0.078993357930
     &    + 0.053629978360*ak)*ak)*ak
     &    - 0.50000000000/ak
     &    +(0.12498593597
     &    +(0.13760497152
     &    +(0.09985066038
     &    + 0.01767148048*ak)*ak)*ak )*alk

      return
      end ! ellek







