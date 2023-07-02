$(document).ready(function() {

  // Update closed state for all sections
  var sectionList = document.getElementsByTagName("section");
  for (i = 0; i < sectionList.length; i++) {
    var sectionId = sectionList[i].id;
    if (localStorage.getItem(sectionId + "-closed-override") === "true") {
      $("#" + sectionId).addClass('closed');
    } else if (localStorage.getItem(sectionId + "-closed-override") === "false") {
      $("#" + sectionId).removeClass('closed');
    } else {
      // Default, do nothing.
    }
  }

  $('section > h1').click(function() {
    $(this).parent().toggleClass('closed');
    // Store closed state of this section for this session
    if (localStorage) {
      localStorage.setItem($(this).parent().attr('id') + "-closed-override", $(this).parent().hasClass('closed'));
    }
  })

});