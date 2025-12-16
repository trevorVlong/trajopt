      subroutine uvsing(x, y, u, v,
     &               nsing,ising,sing,sing2,
     &               xsing,ysing,xsing2,ysing2,asing,rcore, eps)
C      integer ising(*)
C      real sing(*), sing2(*)
C      real xsing(*), ysing(*), xsing2(*), ysing2(*)
C      real asing(*)
C      real rcore(*)

C      include 'lamp.inc'
      complex z1, z2, zlog1, zlog2, delz, zsing, zfun, zfun_z


cf2py   real intent(out) u
cf2py   real intent(out) v

        real u
        real v

Cf2py real intent(IN) x
Cf2py real intent(IN) y
        real  x
        real  y

Cf2py integer intent(IN) nsing
        integer,intent(in) :: nsing

Cf2py integer intent(IN),dimension(nsing):: ising
        integer,intent(in),dimension(nsing)::  ising
Cf2py real intent(IN),depend(sing)  :: nsing=len(sing)
        real, intent(in),dimension(nsing)::  sing
Cf2py real intent(IN),depend(sing2) :: nsing=len(sing2)
        real, intent(in),dimension(nsing)::  sing2
Cf2py real intent(IN),depend(asing) :: nsing=len( asing)
        real, intent(in),dimension(nsing)::  asing

Cf2py real intent(IN) ,depend(xsing)  :: nsing=len(xsing)
Cf2py real intent(IN) ,depend(xsing2)  :: nsing=len(xsing2)
        real, intent(in),dimension(nsing)::   xsing
        real, intent(in),dimension(nsing)::   xsing2

Cf2py real intent(IN) ysing
Cf2py real intent(IN) ysing2
        real, intent(in),dimension(nsing)::   ysing
        real, intent(in),dimension(nsing)::   ysing2

Cf2py real intent(IN),depend(rcore) :: nsing=len(rcore)
        real, intent(in),dimension(nsing)::   rcore

Cf2py real, intent(IN) eps
        real,intent(in):: eps

      data   pi / 3.14159265 /
      data  tpi / 6.28318531 /
      data hopi / 0.159154943 /       !  0.50 / pi
      data qopi / 0.079577471546 /    !  0.25 / pi

      u = 0.
      v = 0.

         OPEN(unit=1,file='data.dat',
     &   POSITION='APPEND',STATUS='UNKNOWN')
         write(1,*) 'x=',x, 'y=',y, 'ising=',ising(1)
         close(1)


      do 100 k = 1, nsing
        cosa = cos(asing(k))
        sina = sin(asing(k))
        xb = x - xsing(k)
        yb = y - ysing(k)

c       rbsq = xb**2 + yb**2
c       rbsq = xb**2 + yb**2 + rcore(k)**2
        rbsq = sqrt((xb**2 + yb**2)**2 + rcore(k)**4)

        rb32 = sqrt(rbsq)**3
        rb52 = sqrt(rbsq)**5
        if(rbsq .lt. eps**2) then
         rbsq = eps**2
         rb32 = eps**3
         rb52 = eps**5
        endif

c----------------------------------------
        if    (ising(k).eq.1) then
c----------- v_inf
         uk   = sing(k)
         vk   = 0.

c----------------------------------------
        elseif(ising(k).eq.2) then
c-----------  source
         if    (ysing(k) .lt. 0.0) then
          uk   = 0.
          vk   = 0.
         elseif(ysing(k) .eq. 0.0) then
          uk   = sing(k)*qopi * xb / rb32
          vk   = sing(k)*qopi * yb / rb32
         else
          call ring( xsing(k),ysing(k), x,y, rcore(k), ug,vg, us,vs)
          uk = us * sing(k) /(tpi*ysing(k))
          vk = vs * sing(k) /(tpi*ysing(k))
         endif

c----------------------------------------
        elseif(ising(k).eq.3) then
c----------- vortex
         if(ysing(k) .le. 0.0) then
          uk = 0.
          vk = 0.
         else
          call ring( xsing(k),ysing(k), x,y, rcore(k), ug,vg, us,vs)
          uk = ug * sing(k)
          vk = vg * sing(k)
         endif

c----------------------------------------
        elseif(ising(k).eq.4) then
c----------- doublet
         if    (ysing(k) .lt. 0.0) then
          uk   = 0.
          vk   = 0.
         elseif(ysing(k) .eq. 0.0) then
c-------- point doublet on axis
          uk   = sing(k)*qopi * (rbsq - 3.0*xb*xb) / rb52
          vk   = sing(k)*qopi * (     - 3.0*xb*yb) / rb52
         else
c-------- ring doublet via two +/- source rings
          xsp = xsing(k) - 0.5*eps*cosa
          ysp = ysing(k) - 0.5*eps*sina
          xsm = xsing(k) + 0.5*eps*cosa
          ysm = ysing(k) + 0.5*eps*sina

          if(ysp .gt. 0.0 .and. ysm .gt. 0.0) then
           call ring( xsp,ysp, x,y, rcore(k), ug,vg, usp,vsp )
           call ring( xsm,ysm, x,y, rcore(k), ug,vg, usm,vsm )
           uk = (usp/ysp - usm/ysm) * (sing(k)/eps)/tpi
           vk = (vsp/ysp - vsm/ysm) * (sing(k)/eps)/tpi
          else
           uk = 0.
           vk = 0.
          endif
         endif

c----------------------------------------
        elseif(ising(k).eq.5 .or. 
     &         ising(k).eq.6      ) then
c------- 2-d source or vortex panel
         x1 = xsing(k)
         y1 = ysing(k)
         x2 = xsing2(k)
         y2 = ysing2(k)

         if(y1 .le. 0.0 .or. y2 .le. 0.0) then
          uk = 0.
          vk = 0.
         else

          call lamp( x1,y1, x2,y2,  x,y, rcore(k),
     &               ug1, vg1, ug2, vg2, 
     &               us1, vs1, us2, vs2 )



          if(ising(k).eq.5) then
c--------- source panel
           uk = (us1 + us2)*sing(k)
           vk = (vs1 + vs2)*sing(k)
          else
c--------- vortex panel
           uk = (ug1 + ug2)*sing(k)
           vk = (vg1 + vg2)*sing(k)
          endif
         endif

        else
         write(*,*) '? uvsing: illegal type', ising(k)
         go to 100
        endif

        u   = u   + uk
        v   = v   + vk

 100  continue

      return
      end ! uvsing

