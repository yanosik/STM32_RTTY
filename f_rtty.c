volatile unsigned char nr_bit =0;

unsigned char send_rtty(char *znak)
{
nr_bit++;
if (nr_bit ==1)
	{
	return 0;
	}
if (nr_bit >1 && nr_bit <10)
	{
	if ((*(znak) >> (nr_bit-2)) & 0x01)
		{
		return 1;
		}
		else
		{
		return 0;
		}
	}

if (nr_bit == 10)
	{
	return 1;
	}
if (nr_bit == 11)
	{
	return 1;
	}

nr_bit =0;
return 2;

}
;
