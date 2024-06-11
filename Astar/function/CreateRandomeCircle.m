function val = CreateRandomeCircle(i,j,CentreX,CentreY,CarreRayon,valPix)

   if ((i-CentreX)*(i-CentreX))+((j-CentreY)*(j-CentreY))<=CarreRayon
       if valPix ==1
           val=0;
       else
           val=1;
       end     
   else

        val=0;
      
   end


end