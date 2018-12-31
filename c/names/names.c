#include <stdio.h>
#include <string.h>
#include <ctype.h>

// $ names
// Who do you want to know about? Lucy
// Lucy is Michael and Sammy's daughter
// 
// $ names
// Who do you want to know about? Sammy
// Sammy is Lucy and Jack's mommy
//
// $ names
// Who do you want to know about? Michael
// Michael is Lucy and Jack's daddy
//
// $ names
// Who do you want to know about? Jack
// Jack is Michael and Sammy's son
//
// $ names
// Who do you want to know about? Gummy Bear
// Gummy bear is in mommy's tummy
//
// $ names
// Who do you want to know about? gobbldygoop
// I'm sorry, I don't know gobbldygoop

int main(void) {
  char buffer[100];
  char buffer2[100];
  printf("Who do you want to know about? ");
  //scanf("%80s", buffer);
  fgets(buffer2, 80, stdin);
  buffer2[strlen(buffer2) - 1] = '\0';
  for (int i = 0; i < 80; i++) {
    buffer[i] = tolower(buffer2[i]);
  }
  if (strcmp(buffer, "lucy") == 0) {
    printf("Lucy is Michael and Sammy's daughter\n");
  } else if (strcmp(buffer, "sammy") == 0) {
    printf("Sammy is Lucy and Jack's mommy\n");
  } else if (strcmp(buffer, "michael") == 0) {
    printf("Michael is Lucy and Jack's daddy\n");
  } else if (strcmp(buffer, "jack") == 0) {
    printf("Jack is Michael and Sammy's son\n");
  } else if (strcmp(buffer, "gummy bear") == 0) {
    printf("Gummy Bear is in mommy's tummy\n");
  } else {
    printf("I'm sorry, I don't know '%s'\n", buffer2);
  }
  return 0;
}
