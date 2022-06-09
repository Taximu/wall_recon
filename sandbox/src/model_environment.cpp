#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
	char sac[512];
	if (argc >= 2)
		sprintf(sac, "./sac %s", argv[1]);
	else
		sprintf(sac, "./sac");

	if (system((char *)sac) == 0)
	{
		if (system("./remove_inclination") == 0)
		{
			if (system("./identify_walls") == 0)
			{
				if (system("./concatenate") == 0)
				{
					if (system("./preparation_for_creating_image") == 0)
					{
						// if(system("./orthogonal_projection") == 0) //is not stable yet, to see results uncomment it
						//{
						if (system("./create_image") == 0)
						{
							puts("Program finished successfully!");
						}
						//}
					}
				}
			}
		}
	}
	return 0;
}
